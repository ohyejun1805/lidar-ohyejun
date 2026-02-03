#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <cmath>
#include <vector>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Vision Msgs
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

// 지면제거 & 공통
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

// 마커
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Dense>

using PointT = pcl::PointXYZI;

// 박스 정보를 담을 구조체
struct BoxInfo {
    float x, y, z;       // 중심 좌표
    float len, wid, hgt; // 크기 (Length, Width, Height)
    float yaw;           // 회전 각도
};

// Autoware 스타일 Search-Based L-Shape Fitting
BoxInfo fitLShape(const pcl::PointCloud<PointT>::Ptr& cluster)
{
    BoxInfo box;
    //z축 미리 계산, 회전과 무관하므로 미리 계산
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    //cluster 안의 x,y,z 최소값과 최대값 계산

    box.z = (min_pt.z + max_pt.z) / 2.0f;
    box.hgt = max_pt.z - min_pt.z;
    //박스 중심 높이와 그냥 높이 계산

    // 2D XY 평면 점들 수집
    std::vector<Eigen::Vector2f> points_2d;
    float cut_height_threshold = min_pt.z + 0.3f;

    for (const auto& p : cluster->points) 
    {
        if(p.z > cut_height_threshold)
        {
            points_2d.push_back(Eigen::Vector2f(p.x, p.y));
        }
    }

    if(points_2d.size() < 3)
    {
        points_2d.clear();
        for(const auto& p : cluster->points) 
        {
            points_2d.push_back(Eigen::Vector2f(p.x, p.y));
        }
    }

    // 변수 초기화
    float min_area = std::numeric_limits<float>::max();
    float best_angle = 0.0f;
    float best_min_x = 0.0f, best_max_x = 0.0f;
    float best_min_y = 0.0f, best_max_y = 0.0f;

    // 각도 정밀 탐색 (0도 ~ 90도)
    // Autoware는 보통 최적화 기법을 쓰지만, Search-Based가 가장 직관적이고 강력함
    for (int angle_step = 0; angle_step < 90; angle_step++) // 1도 간격
    {
        float theta = angle_step * (M_PI / 180.0f); //라디안 변환
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        float current_min_x = std::numeric_limits<float>::max();
        float current_max_x = std::numeric_limits<float>::lowest();
        float current_min_y = std::numeric_limits<float>::max();
        float current_max_y = std::numeric_limits<float>::lowest();

        // 모든 점 회전 변환
        for (const auto& p : points_2d) 
        {
            // 회전 공식: x' = x*cos + y*sin, y' = -x*sin + y*cos
            float rx = p.x() * cos_t + p.y() * sin_t;
            float ry = -p.x() * sin_t + p.y() * cos_t;

            if (rx < current_min_x) current_min_x = rx;
            if (rx > current_max_x) current_max_x = rx;
            if (ry < current_min_y) current_min_y = ry;
            if (ry > current_max_y) current_max_y = ry;
        }

        float width = current_max_x - current_min_x;
        float depth = current_max_y - current_min_y;
        float area = width * depth;

        // 면적이 더 작으면 갱신 (더 타이트한 박스)
        if (area < min_area) 
        {
            min_area = area;
            best_angle = theta;
            best_min_x = current_min_x;
            best_max_x = current_max_x;
            best_min_y = current_min_y;
            best_max_y = current_max_y;
        }
    }

    // 최적 박스 복원
    // 회전된 좌표계에서의 중심
    float cx_rot = (best_min_x + best_max_x) / 2.0f;
    float cy_rot = (best_min_y + best_max_y) / 2.0f;
    
    // 원래 좌표계로 역회전
    float cos_best = std::cos(best_angle);
    float sin_best = std::sin(best_angle);

    box.x = cx_rot * cos_best - cy_rot * sin_best;
    box.y = cx_rot * sin_best + cy_rot * cos_best;
    //원점 기준으로 세타만큼 회전해서 원래 좌표계로 역회전
    
    // 크기 설정
    float dim_x = best_max_x - best_min_x;
    float dim_y = best_max_y - best_min_y;

    // Heading 정리 (긴 변을 차량의 앞뒤로 가정)
    //그러면 내 차선 앞에 있는 차량은? 그건 tracking에서 속도 방향으로 보정.
    if (dim_x < dim_y) 
    {
        std::swap(dim_x, dim_y);
        best_angle += (M_PI / 2.0f); // 90도 회전
    }

    box.len = dim_x;
    box.wid = dim_y;
    box.yaw = best_angle;

    return box;
}

class GigachaLidarClustering
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    
    ros::Publisher cloud_origin_pub_;
    ros::Publisher cloud_down_pub_;
    ros::Publisher cloud_crop_pub_;
    ros::Publisher cloud_ground_removed_pub_; 
    ros::Publisher cloud_cluster_pub_;
    ros::Publisher bbox_pub_; 
    ros::Publisher marker_pub_;

    float voxel_size_;
    float roi_min_x_, roi_max_x_;
    float roi_min_y_, roi_max_y_;
    float roi_min_z_, roi_max_z_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

    bool publish_origin_;
    bool publish_down_;
    bool publish_crop_;
    bool publish_clustered_;

public:
    GigachaLidarClustering() : nh_("~")
    {
        ROS_INFO("GIGACHA LiDAR Clustering (Autoware L-Shape) Starting...");
        
        nh_.param<float>("voxel_size", voxel_size_, 0.12f);
        nh_.param<float>("roi_min_x", roi_min_x_, -20.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 30.0f);
        nh_.param<float>("roi_min_y", roi_min_y_, -10.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 10.0f);
        nh_.param<float>("roi_min_z", roi_min_z_, -5.0f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 2.0f);
        
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.55f);
        nh_.param<int>("min_cluster_size", min_cluster_size_, 3);
        nh_.param<int>("max_cluster_size", max_cluster_size_, 5000);

        nh_.param<bool>("publish_origin", publish_origin_, true);
        nh_.param<bool>("publish_down", publish_down_, true);
        nh_.param<bool>("publish_crop", publish_crop_, true);
        nh_.param<bool>("publish_clustered", publish_clustered_, true);

        cloud_origin_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_origin", 1);
        cloud_down_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_downsampled", 1);
        cloud_crop_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_roi", 1);
        cloud_ground_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_ground_removed", 1);
        cloud_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_clustered", 1);
        bbox_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/gigacha/lidar/markers", 1);
        
        lidar_sub_ = nh_.subscribe("/lidar3D", 1, &GigachaLidarClustering::lidarCallback, this);
        
        ROS_INFO("Initialized.");
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_down(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);

        pcl::fromROSMsg(*msg, *cloud_origin);

        if (cloud_origin->empty()) return;

        // 1. Voxel Grid
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_origin);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*cloud_down);

        // 2. Crop Box
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_down);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        // 3. Ground Removal (RANSAC)
        pcl::PointCloud<PointT>::Ptr cloud_obstacles_total(new pcl::PointCloud<PointT>);
        float zone_limits[4] = {roi_min_x_, 10.0f, 25.0f, roi_max_x_};
        float zone_thresholds[3] = {0.15f, 0.20f, 0.25f};

        for(int i = 0; i < 3; i++) 
        {
            pcl::PointCloud<PointT>::Ptr cloud_zone(new pcl::PointCloud<PointT>);
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud_crop);             
            pass.setFilterFieldName("x");               
            pass.setFilterLimits(zone_limits[i], zone_limits[i+1]); 
            pass.filter(*cloud_zone);                   

            if (cloud_zone->empty()) continue; 

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
            pcl::SACSegmentation<PointT> seg;
            
            seg.setOptimizeCoefficients(true);          
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
            seg.setMethodType(pcl::SAC_RANSAC);         
            seg.setMaxIterations(200);                  
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); 
            seg.setEpsAngle(12.0f * (M_PI / 180.0f));   
            seg.setDistanceThreshold(zone_thresholds[i]); 

            seg.setInputCloud(cloud_zone);
            seg.segment(*inliers, *coefficients);   
            
            pcl::PointCloud<PointT>::Ptr cloud_zone_obstacle(new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud_zone);
            extract.setIndices(inliers);                
            extract.setNegative(true);                  
            extract.filter(*cloud_zone_obstacle);       

            *cloud_obstacles_total += *cloud_zone_obstacle;
        }

        sensor_msgs::PointCloud2 out_obs;
        pcl::toROSMsg(*cloud_obstacles_total, out_obs); 
        out_obs.header = msg->header;
        cloud_ground_removed_pub_.publish(out_obs);

        if (cloud_obstacles_total->empty()) return;

        // Clustering (Z축 제거 후 2D로 수행)
        pcl::PointCloud<PointT>::Ptr cloud_2d(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_obstacles_total, *cloud_2d);
        for (auto& p : cloud_2d->points) { p.z = 0.0f; } // z축 제거

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_2d); 

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> clustering;
        clustering.setInputCloud(cloud_2d); 
        clustering.setClusterTolerance(cluster_tolerance_); 
        clustering.setMinClusterSize(min_cluster_size_);
        clustering.setMaxClusterSize(max_cluster_size_);
        clustering.setSearchMethod(tree);
        clustering.extract(cluster_indices);

        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        cloud_clustered->header = cloud_obstacles_total->header;
        cloud_clustered->is_dense = false;

        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header; 
        
        visualization_msgs::MarkerArray marker_array; 

        int cluster_id = 0;
        
        // Box Fitting Loop
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            if (indices.indices.size() < 5) continue; // 점 너무 적으면 무시
            
            for (const auto &idx : indices.indices)
            {
                PointT p = cloud_obstacles_total->points[idx];
                p.intensity = static_cast<float>(cluster_id % 100); 
                cluster->points.push_back(p);
                cloud_clustered->points.push_back(p);
            }
            
            // Autoware L-Shape Fitting 호출
            BoxInfo box = fitLShape(cluster);

            if(box.hgt < 0.5f)
            {
                continue;
            }

            // 사람 + 차 + 버스 필터링 
            float min_side = std::min(box.len, box.wid); // 폭
            float max_side = std::max(box.len, box.wid); // 길이

            // 기본 노이즈 제거
            if (max_side < 0.2f) continue;   // 20cm 미만
            if (max_side > 20.0f) continue;  // 20m 초과

            //조건
            
            // [사람] 
            bool is_normal_person = (min_side >= 0.1f && min_side < 1.0f) && 
                             (max_side >= 0.1f && max_side < 1.2f);

            bool is_wide_person = (max_side >= 1.2f && max_side < 2.0f)&&(min_side <0.6f);

            bool is_person = is_normal_person || is_wide_person;

            // [승용차] 폭 1.0~2.5m, 길이 2.0~6.0m
            bool is_normal_car = (min_side >= 1.0f && min_side < 2.5f) && 
                          (max_side >= 2.0f && max_side < 6.0f);
            bool is_partial_car = (min_side >= 1.0f && min_side <2.5f) && 
                          (max_side >= 0.3f && max_side < 2.0f);

            bool is_car = is_normal_car || is_partial_car;

            // [폭 2.0~3.5m, 길이 5.5~19.0m
            bool is_large = (min_side >= 2.0f && min_side < 3.5f) && 
                            (max_side >= 5.5f && max_side < 19.0f);
            
            // "내 차 중심 좌우 4m 이내"에 있어야만 버스로 인정 (벽 제거 꼼수)
            bool is_on_road = std::abs(box.y) < 4.0f; 
            
            bool is_bus = is_large && is_on_road;

            // 최종 판정: 셋 중 하나라도 아니면 삭제
            if (!is_person && !is_car && !is_bus) 
            {
                continue; 
            }

            // Detection3D 메시지 생성
            vision_msgs::Detection3D detection;
            detection.header = detection_array.header;
            detection.bbox.center.position.x = box.x;
            detection.bbox.center.position.y = box.y;
            detection.bbox.center.position.z = box.z;

            // Yaw -> Quaternion 변환
            Eigen::Quaternionf q;
            q = Eigen::AngleAxisf(box.yaw, Eigen::Vector3f::UnitZ());
            detection.bbox.center.orientation.x = q.x();
            detection.bbox.center.orientation.y = q.y();
            detection.bbox.center.orientation.z = q.z();
            detection.bbox.center.orientation.w = q.w();

            detection.bbox.size.x = box.len;
            detection.bbox.size.y = box.wid;
            detection.bbox.size.z = box.hgt;

            vision_msgs::ObjectHypothesisWithPose hypothesis;
            hypothesis.id = 0; 
            hypothesis.score = 1.0; 
            detection.results.push_back(hypothesis);
            detection_array.detections.push_back(detection);

            // 마커 생성 (텍스트 없음, 큐브만 생성)
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "bbox"; // 네임스페이스 변경 (잔상 방지용)
            marker.id = cluster_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detection.bbox.center;
            marker.pose.orientation = detection.bbox.center.orientation;
            marker.scale = detection.bbox.size;
            marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; marker.color.a = 0.5f;
            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);

            cluster_id++;
        }

        // Publish
        if (publish_origin_) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_origin, out);
            out.header = msg->header;
            cloud_origin_pub_.publish(out);
        }
        if (publish_down_) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_down, out);
            out.header = msg->header;
            cloud_down_pub_.publish(out);
        }
        if (publish_crop_) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_crop, out);
            out.header = msg->header;
            cloud_crop_pub_.publish(out);
        }
        
        if (publish_clustered_) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_clustered, out);
            out.header = msg->header;
            cloud_cluster_pub_.publish(out);
        }

        bbox_pub_.publish(detection_array);
        marker_pub_.publish(marker_array);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_clustering");
    GigachaLidarClustering node;
    ros::spin();
    return 0;
}