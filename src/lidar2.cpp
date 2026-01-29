#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// vision_msgs
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

// 지면제거
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// 마커 헤더
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using PointT = pcl::PointXYZI;

//L-Shape로 제일 fit한 rect (대각선 박스 방지)
cv::RotatedRect getBestFitRect(const std::vector<cv::Point2f>& points)
{
    if (points.size() < 5) return cv::minAreaRect(points);

    float best_angle = 0.0f;
    float min_area = std::numeric_limits<float>::max();
    
    cv::Point2f center(0, 0);
    for (const auto& p : points) center += p;
    center.x /= points.size();
    center.y /= points.size();

    // 1. 각도 탐색 (0~90도) 2도씩
    for (float angle = 0.0f; angle < 90.0f; angle += 2.0f)
    {
        float theta = angle * CV_PI / 180.0f;
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto& p : points)
        {
            float tx = p.x - center.x;
            float ty = p.y - center.y;
            float rx = tx * cos_t - ty * sin_t;
            float ry = tx * sin_t + ty * cos_t;

            if (rx < min_x) min_x = rx;
            if (rx > max_x) max_x = rx;
            if (ry < min_y) min_y = ry;
            if (ry > max_y) max_y = ry;
        }

        float area = (max_x - min_x) * (max_y - min_y);

        if (area < min_area)
        {
            min_area = area;
            best_angle = angle;
        }
    }

    // 2. 최적 각도로 최종 박스 다시 계산
    float theta = best_angle * CV_PI / 180.0f;
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& p : points)
    {
        float tx = p.x - center.x;
        float ty = p.y - center.y;
        float rx = tx * cos_t - ty * sin_t;
        float ry = tx * sin_t + ty * cos_t;

        if (rx < min_x) min_x = rx;
        if (rx > max_x) max_x = rx;
        if (ry < min_y) min_y = ry;
        if (ry > max_y) max_y = ry;
    }

    float width = max_x - min_x;
    float height = max_y - min_y;
    
    return cv::RotatedRect(center, cv::Size2f(width, height), best_angle);
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
    //제어기용 지면 제거 후 전체 장애물
    ros::Publisher cloud_cluster_pub_;
    ros::Publisher bbox_pub_; 
    //트래킹용 동적 장애물
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
        ROS_INFO("GIGACHA LiDAR Clustering Node Starting...");
        
        nh_.param<float>("voxel_size", voxel_size_, 0.12f);
        nh_.param<float>("roi_min_x", roi_min_x_, -10.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 30.0f);
        nh_.param<float>("roi_min_y", roi_min_y_, -6.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 6.0f);
        nh_.param<float>("roi_min_z", roi_min_z_, -5.0f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 0.3f);
        
        // 차와 벽을 분리 위해 Tolerance를 줄임
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.25f);
        nh_.param<int>("min_cluster_size", min_cluster_size_, 5);
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

    ~GigachaLidarClustering()//소멸자
    {
        ROS_INFO("GIGACHA LiDAR Clustering Node Terminated");
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_down(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);

        pcl::fromROSMsg(*msg, *cloud_origin);

        if (cloud_origin->empty()) return;

        // voxel
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_origin);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*cloud_down);

        // crop
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_down);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        // 3 Ransac
        pcl::PointCloud<PointT>::Ptr cloud_obstacles_total(new pcl::PointCloud<PointT>);
        float zone_limits[4] = {roi_min_x_, 10.0f, 25.0f, roi_max_x_};
        float zone_thresholds[3] = {0.15f, 0.25f, 0.35f};

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

        if (cloud_obstacles_total->empty()) return;

        //2D로 점 눌러서 clustering 가짜 2D 점으로 묶고, 원본 점으로 box 침.
        pcl::PointCloud<PointT>::Ptr cloud_2d(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_obstacles_total, *cloud_2d);

        for (auto& p : cloud_2d->points) { p.z = 0.0f; }
        //모든 점을 z=0으로 바꿈.

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_2d); // 2D 데이터 입력

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> clustering;
        clustering.setInputCloud(cloud_2d); // 2D 데이터 입력
        clustering.setClusterTolerance(cluster_tolerance_); 
        clustering.setMinClusterSize(min_cluster_size_);
        clustering.setMaxClusterSize(max_cluster_size_);
        clustering.setSearchMethod(tree);
        clustering.extract(cluster_indices);

        // 결과 담을 변수들
        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        cloud_clustered->header = cloud_obstacles_total->header;
        cloud_clustered->is_dense = false;

        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header; 
        
        visualization_msgs::MarkerArray marker_array;
        //marker array 생성

        int cluster_id = 0;
        
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            
            for (const auto &idx : indices.indices)
            {
                // 점 좌표는 원본(cloud_obstacles_total)에서 가져옴!
                PointT p = cloud_obstacles_total->points[idx];
                p.intensity = static_cast<float>(cluster_id % 100); 
                cluster->points.push_back(p);
                cloud_clustered->points.push_back(p);
            }
            
            // Z축 범위 계산
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();
            for (const auto& p : cluster->points) {
                if (p.z < min_z) min_z = p.z;
                if (p.z > max_z) max_z = p.z;
            }

            // 2D 박스 피팅
            std::vector<cv::Point2f> points_2d;
            for (const auto& p : cluster->points) {
                points_2d.push_back(cv::Point2f(p.x, p.y));
            }

            cv::RotatedRect rect = getBestFitRect(points_2d);
            //points_2d로 getbestfitrect 찾음.

            float center_x = rect.center.x;
            float center_y = rect.center.y;
            float center_z = (min_z + max_z) / 2.0f; 

            float size_x = rect.size.width;
            float size_y = rect.size.height;
            float size_z = max_z - min_z;
            float angle_deg = rect.angle;

            // 박스 정렬
            if (size_x < size_y) {
                std::swap(size_x, size_y);
                angle_deg += 90.0f;
            }
            // 내 앞에 달리고 있는 차는 폭이 넓고 짧은 변이 너무 얇으면,
            // 차의 옆모습이 아니고, 뒷모습이라고 인식! 다시 swap
            if (size_x > 1.2f && size_y < 0.8f) {
                 std::swap(size_x, size_y); 
                 angle_deg += 90.0f;
            }

            //도보, 건물, 큰 차 구분.
            //여기서 걸러지면 tracking 안함.

            // 바닥에 붙어있고(-1.0m 이하), 높이가 낮으면(40cm 미만) 도보
            if (min_z < -1.0f && size_z < 0.4f) continue; 

            // 대각선과 짧은 옆면 구함.
            float diagonal = std::sqrt(size_x*size_x + size_y*size_y);
            float min_side = std::min(size_x, size_y);

            //나랑 좌우로 얼마나 떨어져 있냐(abs는 절대값)
            float abs_y = std::abs(center_y);

            // 대각선 6m 이상
            if (diagonal > 6.0f) 
            {
                //두께가 얇음
                if (min_side < 0.5f) continue;

                if (min_side > 3.5f) continue;

                //위치가 좌우로 5m 밖임.
                if (abs_y > 5.0f) continue; 
            }

            //높이 5m
            if (size_z > 5.0f) continue; 

            //너무 작은 노이즈
            if (size_x * size_y < 0.2f) continue;

            float yaw = angle_deg * M_PI / 180.0f; 
            Eigen::Quaternionf q;
            q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

            // 메시지 생성
            vision_msgs::Detection3D detection;
            detection.header = detection_array.header;
            detection.bbox.center.position.x = center_x;
            detection.bbox.center.position.y = center_y;
            detection.bbox.center.position.z = center_z;
            detection.bbox.center.orientation.x = q.x();
            detection.bbox.center.orientation.y = q.y();
            detection.bbox.center.orientation.z = q.z();
            detection.bbox.center.orientation.w = q.w();
            detection.bbox.size.x = size_x;
            detection.bbox.size.y = size_y;
            detection.bbox.size.z = size_z;

            vision_msgs::ObjectHypothesisWithPose hypothesis;
            hypothesis.id = 0; 
            hypothesis.score = 1.0; 
            detection.results.push_back(hypothesis);

            detection_array.detections.push_back(detection);

            // 마커 생성
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "cluster_box";
            marker.id = cluster_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detection.bbox.center;
            marker.pose.orientation = detection.bbox.center.orientation;
            marker.scale = detection.bbox.size;
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; 
            marker.color.a = 0.5f;
            marker.lifetime = ros::Duration(0.1);
            
            marker_array.markers.push_back(marker);

            cluster_id++;
        }

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

        //제어용 지면제거만 한 모든 점들.
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(*cloud_obstacles_total, out); 
        out.header = msg->header;
        cloud_ground_removed_pub_.publish(out);

        if (publish_clustered_) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_clustered, out);
            out.header = msg->header;
            cloud_cluster_pub_.publish(out);
        }

        // 여기에는 오직 필터링을 통과한 물체만 들어있음!
        bbox_pub_.publish(detection_array);
        
        // Rviz 마커 전송
        marker_pub_.publish(marker_array);
        
        ROS_DEBUG("Clusters: %zu", cluster_indices.size());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_clustering");
    GigachaLidarClustering node;
    ros::spin();
    return 0;
}