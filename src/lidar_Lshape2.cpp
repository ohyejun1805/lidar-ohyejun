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

// 공통
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

// 마커
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Dense>

// [Patchwork++] 헤더
#include <patchworkpp/patchworkpp.hpp> 

using PointT = pcl::PointXYZI;

struct BoxInfo {
    float x, y, z;       
    float len, wid, hgt; 
    float yaw;           
};

// Autoware 스타일 Search-Based L-Shape Fitting
BoxInfo fitLShape(const pcl::PointCloud<PointT>::Ptr& cluster)
{
    BoxInfo box;
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    box.z = (min_pt.z + max_pt.z) / 2.0f;
    box.hgt = max_pt.z - min_pt.z;

    std::vector<Eigen::Vector2f> points_2d;
    float cut_height_threshold = min_pt.z + 0.3f; 

    for (const auto& p : cluster->points) 
    {
        if(p.z > cut_height_threshold)
            points_2d.push_back(Eigen::Vector2f(p.x, p.y));
    }

    if(points_2d.size() < 3)
    {
        points_2d.clear();
        for(const auto& p : cluster->points) 
            points_2d.push_back(Eigen::Vector2f(p.x, p.y));
    }

    float min_area = std::numeric_limits<float>::max();
    float best_angle = 0.0f;
    float best_min_x = 0.0f, best_max_x = 0.0f;
    float best_min_y = 0.0f, best_max_y = 0.0f;

    for (int angle_step = 0; angle_step < 90; angle_step++) 
    {
        float theta = angle_step * (M_PI / 180.0f);
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        float current_min_x = std::numeric_limits<float>::max();
        float current_max_x = std::numeric_limits<float>::lowest();
        float current_min_y = std::numeric_limits<float>::max();
        float current_max_y = std::numeric_limits<float>::lowest();

        for (const auto& p : points_2d) 
        {
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

    float cx_rot = (best_min_x + best_max_x) / 2.0f;
    float cy_rot = (best_min_y + best_max_y) / 2.0f;
    
    float cos_best = std::cos(best_angle);
    float sin_best = std::sin(best_angle);

    box.x = cx_rot * cos_best - cy_rot * sin_best;
    box.y = cx_rot * sin_best + cy_rot * cos_best;
    
    float dim_x = best_max_x - best_min_x;
    float dim_y = best_max_y - best_min_y;

    if (dim_x < dim_y) 
    {
        std::swap(dim_x, dim_y);
        best_angle += (M_PI / 2.0f); 
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
    ros::Publisher cloud_ground_removed_pub_; 
    ros::Publisher cloud_cluster_pub_;        
    ros::Publisher bbox_pub_;                 
    ros::Publisher marker_pub_;               

    // 템플릿 클래스 선언
    boost::shared_ptr<PatchWorkpp<PointT>> patchwork_ptr_; 

    float voxel_size_;
    float roi_min_x_, roi_max_x_;
    float roi_min_y_, roi_max_y_;
    float roi_min_z_, roi_max_z_;
    
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

public:
    GigachaLidarClustering() : nh_("~")
    {
        ROS_INFO("GIGACHA LiDAR Clustering (With Patchwork++) Init...");
        
        nh_.param<float>("voxel_size", voxel_size_, 0.15f);
        nh_.param<float>("roi_min_x", roi_min_x_, -20.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 30.0f);
        nh_.param<float>("roi_min_y", roi_min_y_, -7.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 7.0f);
        nh_.param<float>("roi_min_z", roi_min_z_, -2.5f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 2.5f);
        
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.6f);
        nh_.param<int>("min_cluster_size", min_cluster_size_, 10);
        nh_.param<int>("max_cluster_size", max_cluster_size_, 5000);

        // [수정] 복잡한 setParam 다 제거함 (launch 파일에서 관리)

        // 객체 생성 (NodeHandle 포인터 전달)
        patchwork_ptr_.reset(new PatchWorkpp<PointT>(&nh_));

        cloud_origin_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_origin", 1);
        cloud_ground_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_ground_removed", 1);
        cloud_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_clustered", 1);
        bbox_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/gigacha/lidar/markers", 1);
        
        lidar_sub_ = nh_.subscribe("/lidar3D", 1, &GigachaLidarClustering::lidarCallback, this);
        
        ROS_INFO("Initialized Successfully.");
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud_origin);

        if (cloud_origin->empty()) return;

        // --- Step 1: Patchwork++ Ground Removal ---
        pcl::PointCloud<PointT> cloud_ground, cloud_nonground;
        
        double time_taken;
        // estimate_ground 호출
        patchwork_ptr_->estimate_ground(*cloud_origin, cloud_ground, cloud_nonground, time_taken);

        pcl::PointCloud<PointT>::Ptr cloud_obstacles(new pcl::PointCloud<PointT>);
        *cloud_obstacles = cloud_nonground; 

        sensor_msgs::PointCloud2 out_obs;
        pcl::toROSMsg(*cloud_obstacles, out_obs); 
        out_obs.header = msg->header;
        cloud_ground_removed_pub_.publish(out_obs);

        if (cloud_obstacles->empty()) return;

        // --- Step 2: ROI (Crop Box) ---
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_obstacles);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        if (cloud_crop->empty()) return;
        // =========================================================
        // [추가] 사용자 아이디어 적용: 거리별 Z축 커팅 (Voxel 전처리)
        // "가까운 건 과감하게 바닥 30cm 자르고, 먼 건 살린다"
        // =========================================================
        
        pcl::PointCloud<PointT>::Ptr cloud_z_filtered(new pcl::PointCloud<PointT>);
        cloud_z_filtered->reserve(cloud_crop->points.size());

        float dist_threshold = 20.0f; // 20m 기준
        
        // 센서 높이 1.55m 가정 시, 바닥은 z = -1.55
        // 여기서 30cm 위를 자르려면: -1.55 + 0.3 = -1.25보다 큰 것만 남김
        // (사용자 환경에 맞춰서 -1.2 ~ -1.3 사이로 조절하세요)
        float z_cut_threshold = -1.25f; 

        for (const auto& p : cloud_crop->points)
        {
            float dist = std::sqrt(p.x * p.x + p.y * p.y);

            if (dist < dist_threshold)
            {
                // [가까운 영역] 바닥 노이즈랑 뭉치지 않게 하단 30cm 삭제
                if (p.z > z_cut_threshold) 
                {
                    cloud_z_filtered->points.push_back(p);
                }
            }
            else
            {
                // [먼 영역] 점이 듬성듬성하므로 바닥 쪽이라도 소중하게 살림
                cloud_z_filtered->points.push_back(p);
            }
        }
        
        // 필터링 된 결과를 Voxel Grid 입력으로 사용하기 위해 덮어씌움
        *cloud_crop = *cloud_z_filtered;

        // --- Step 3: Voxel Grid Downsampling ---
        pcl::PointCloud<PointT>::Ptr cloud_final(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_crop);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_ * 2.5f);
        voxel_filter.filter(*cloud_final);

        if (cloud_final->empty()) return;

        // --- Step 4: Clustering ---
        pcl::PointCloud<PointT>::Ptr cloud_2d(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_final, *cloud_2d);
        for (auto& p : cloud_2d->points) { p.z = 0.0f; } 

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

        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header; 
        
        visualization_msgs::MarkerArray marker_array; 
        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        
        pcl_conversions::toPCL(msg->header, cloud_clustered->header);

        int cluster_id = 0;
        
        // --- Step 5: Bounding Box Fitting ---
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            for (const auto &idx : indices.indices)
            {
                PointT p = cloud_final->points[idx];
                p.intensity = static_cast<float>(cluster_id % 100); 
                cluster->points.push_back(p);
                cloud_clustered->points.push_back(p);
            }
            
            BoxInfo box = fitLShape(cluster);

            float min_side = std::min(box.len, box.wid); 
            float max_side = std::max(box.len, box.wid); 

            if (max_side < 0.2f) continue;  
            if (max_side > 18.0f) continue; 
            if (box.hgt < 0.2f) continue; 

            bool is_person = (min_side < 1.0f) && (max_side < 1.2f);
            bool is_car = (min_side >= 1.0f && min_side < 2.5f) && (max_side >= 2.0f && max_side < 6.0f);
            bool is_large = (min_side >= 2.0f && min_side < 3.5f) && (max_side >= 5.5f && max_side < 19.0f);
            bool is_on_road = std::abs(box.y) < 4.5f; 
            bool is_bus = is_large && is_on_road;

            if (!is_person && !is_car && !is_bus) continue; 

            vision_msgs::Detection3D detection;
            detection.header = detection_array.header;
            detection.bbox.center.position.x = box.x;
            detection.bbox.center.position.y = box.y;
            detection.bbox.center.position.z = box.z;

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

            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "bbox"; 
            marker.id = cluster_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detection.bbox.center;
            marker.pose.orientation = detection.bbox.center.orientation;
            marker.scale = detection.bbox.size;
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.4f;
            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);

            cluster_id++;
        }

        cloud_cluster_pub_.publish(*cloud_clustered);
        bbox_pub_.publish(detection_array);
        marker_pub_.publish(marker_array);

        cloud_origin_pub_.publish(*cloud_origin);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_clustering");
    GigachaLidarClustering node;
    ros::spin();
    return 0;
}