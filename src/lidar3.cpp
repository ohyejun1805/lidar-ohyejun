#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <cmath>

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

// Convex Hull + Search Fitting
cv::RotatedRect getBestFitRect(const std::vector<cv::Point2f>& points)
{
    if (points.size() < 5) return cv::minAreaRect(points);

    // 1. Convex Hull (외곽선 따기)
    std::vector<cv::Point2f> hull;
    cv::convexHull(points, hull);

    if (hull.size() < 3) return cv::minAreaRect(points);

    float best_angle = 0.0f;
    float min_score = std::numeric_limits<float>::max(); // Score 사용
    
    cv::Point2f center(0, 0);
    for (const auto& p : hull) center += p;
    center.x /= hull.size();
    center.y /= hull.size();

    // 2. 각도 정밀 탐색 (0~90도)
    for (float angle = 0.0f; angle < 90.0f; angle += 1.0f)
    {
        float theta = angle * CV_PI / 180.0f;
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto& p : hull)
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
        
        // Soft Bias (소프트 가중치)
        // 0도, 90도 근처면 점수를 깎아줌.
        // 대각선이면 점수를 높임(패널티).
        // 이렇게 하면 "애매한 L자"는 0도로 붙고, "진짜 회전하는 차"는 대각선 유지.
        float score = area;
        
        // 0도 or 90도 근처면 Area를 0.8배로 쳐줌 (우선순위 부여)
        if (angle < 10.0f || angle > 80.0f) {
            score *= 0.8f; 
        }

        if (score < min_score)
        {
            min_score = score;
            best_angle = angle;
        }
    }

    // 최적 각도 재계산
    float theta = best_angle * CV_PI / 180.0f;
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& p : hull)
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
        ROS_INFO("GIGACHA LiDAR Clustering Node Starting...");
        
        nh_.param<float>("voxel_size", voxel_size_, 0.12f);
        nh_.param<float>("roi_min_x", roi_min_x_, -10.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 30.0f);
        nh_.param<float>("roi_min_y", roi_min_y_, -5.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 5.0f);
        nh_.param<float>("roi_min_z", roi_min_z_, -5.0f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 0.3f);
        
        // [수정] 0.55m (컨테이너 분리 + 차체 연결 최적값)
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.55f);
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

    ~GigachaLidarClustering() {}

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

        // 3. Ground Removal
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

        // ROI Slicing: 바닥에서 30cm 위(-1.5m) 이상만 트래킹
        pcl::PointCloud<PointT>::Ptr cloud_tracking_input(new pcl::PointCloud<PointT>);
        for (const auto& p : cloud_obstacles_total->points) {
            if (p.z > -1.5f) { 
                cloud_tracking_input->push_back(p);
            }
        }

        if (cloud_tracking_input->empty()) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_obstacles_total, out); 
            out.header = msg->header;
            cloud_ground_removed_pub_.publish(out);
            return;
        }

        // 4. 2D Projection
        pcl::PointCloud<PointT>::Ptr cloud_2d(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud_tracking_input, *cloud_2d);

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

        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        cloud_clustered->header = cloud_obstacles_total->header;
        cloud_clustered->is_dense = false;

        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header; 
        
        visualization_msgs::MarkerArray marker_array; 

        int cluster_id = 0;
        
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            if (indices.indices.size() < 10) continue; 
            
            for (const auto &idx : indices.indices)
            {
                PointT p = cloud_tracking_input->points[idx];
                p.intensity = static_cast<float>(cluster_id % 100); 
                cluster->points.push_back(p);
                cloud_clustered->points.push_back(p);
            }
            
            float max_z = std::numeric_limits<float>::lowest();
            for (const auto& p : cluster->points) 
            {
                if (p.z > max_z) max_z = p.z;
            }
            float min_z = -1.8f; 

            std::vector<cv::Point2f> points_2d;
            for (const auto& p : cluster->points) 
            {
                points_2d.push_back(cv::Point2f(p.x, p.y));
            }

            cv::RotatedRect rect = getBestFitRect(points_2d);

            float center_x = rect.center.x;
            float center_y = rect.center.y;
            float center_z = (min_z + max_z) / 2.0f; 

            float size_x = rect.size.width;
            float size_y = rect.size.height;
            float size_z = max_z - min_z;
            float angle_deg = rect.angle;

            // [수정] 무조건 긴 변을 Heading으로 (Rear View 제거)
            if (size_x < size_y) 
            {
                std::swap(size_x, size_y);
                angle_deg += 90.0f;
            }


            // 필터링
            float diagonal = std::sqrt(size_x*size_x + size_y*size_y);
            float min_side = std::min(size_x, size_y);
            float abs_y = std::abs(center_y);

            if (diagonal > 6.0f) 
            {
                if (min_side < 0.5f) continue; 
                if (min_side > 3.5f) continue; 
                if (abs_y > 5.0f) continue;    
                if (size_x > 15.0f) continue;
            }

            if (size_z > 4.5f) continue; 
            if (size_x * size_y < 0.05f) continue;

            float yaw = angle_deg * M_PI / 180.0f; 
            Eigen::Quaternionf q;
            q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

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

        bbox_pub_.publish(detection_array);
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