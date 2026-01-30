#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <cmath>
#include <vector>

// PCL Headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

// Messages
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <visualization_msgs/MarkerArray.h>

using PointT = pcl::PointXYZI;

class GigachaLidarCone
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    
    ros::Publisher cloud_roi_pub_; 
    ros::Publisher marker_pub_;    
    ros::Publisher bbox_pub_;      

    float roi_min_x_, roi_max_x_;
    float roi_min_y_, roi_max_y_;
    float roi_min_z_, roi_max_z_;
    
    int min_cluster_size_;
    int max_cluster_size_;
    float cluster_tolerance_;

public:
    GigachaLidarCone() : nh_("~")
    {
        ROS_INFO("GIGACHA LiDAR Cone Detection (No Color, TF 1.0m) Starting...");
        
        // [설정 1] ROI (가까운 영역만)
        nh_.param<float>("roi_min_x", roi_min_x_, 0.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 8.0f); // 8m 이내
        nh_.param<float>("roi_min_y", roi_min_y_, -4.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 4.0f);
        
        // [설정 2] Z축 Slicing (TF 1.0m 기준)
        // 바닥(-1.0m)보다 40cm 위인 -0.6m부터 잘라서 꼬깔 머리만 남김
        nh_.param<float>("roi_min_z", roi_min_z_, -0.6f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 0.5f); 

        // [설정 3] 클러스터링
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.3f); 
        nh_.param<int>("min_cluster_size", min_cluster_size_, 3); 
        nh_.param<int>("max_cluster_size", max_cluster_size_, 300);

        cloud_roi_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_roi", 1);
        bbox_pub_      = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        marker_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/gigacha/lidar/markers", 1);
        
        // 토픽 이름 설정 (실행 시 _lidar_topic:=... 으로 변경 가능)
        std::string lidar_topic;
        nh_.param<std::string>("lidar_topic", lidar_topic, "/ouster/points");
        lidar_sub_ = nh_.subscribe(lidar_topic, 1, &GigachaLidarCone::lidarCallback, this);
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);

        pcl::fromROSMsg(*msg, *cloud_origin);
        if (cloud_origin->empty()) return;

        // 1. CropBox (바닥 제거)
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_origin);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        // 디버깅용 (잘린 점들 확인)
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(*cloud_crop, out);
        out.header = msg->header;
        cloud_roi_pub_.publish(out);

        if (cloud_crop->empty()) return;

        // 2. Clustering
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_crop); 

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> clustering;
        clustering.setInputCloud(cloud_crop); 
        clustering.setClusterTolerance(cluster_tolerance_); 
        clustering.setMinClusterSize(min_cluster_size_);
        clustering.setMaxClusterSize(max_cluster_size_);
        clustering.setSearchMethod(tree);
        clustering.extract(cluster_indices);

        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header;
        
        visualization_msgs::MarkerArray marker_array; 
        
        // 마커 초기화 (잔상 제거)
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        int cluster_id = 0;

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            
            // Intensity 계산 루프 삭제함 (단순화)
            for (const auto &idx : indices.indices) {
                cluster->points.push_back(cloud_crop->points[idx]);
            }

            // Min/Max 계산
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float size_x = max_pt[0] - min_pt[0];
            float size_y = max_pt[1] - min_pt[1];
            
            // [조건 필터] 뚱뚱하면 꼬깔 아님 (0.6m 이상)
            if (size_x > 0.6f || size_y > 0.6f) continue;

            float center_x = (min_pt[0] + max_pt[0]) / 2.0f;
            float center_y = (min_pt[1] + max_pt[1]) / 2.0f;
            
            // [★ 시각화 높이 보정]
            // TF 1.0m 기준이므로 바닥은 -1.0m
            float floor_z = -1.0f; 
            float top_z = max_pt[2]; // 감지된 점의 제일 높은 곳
            
            float marker_z = (floor_z + top_z) / 2.0f; // 원통의 중심
            float marker_h = top_z - floor_z;          // 원통의 길이

            // Detection 메시지
            vision_msgs::Detection3D detection;
            detection.header = msg->header;
            detection.bbox.center.position.x = center_x;
            detection.bbox.center.position.y = center_y;
            detection.bbox.center.position.z = marker_z;
            detection.bbox.center.orientation.w = 1.0;
            detection.bbox.size.x = 0.4f; 
            detection.bbox.size.y = 0.4f;
            detection.bbox.size.z = marker_h;
            
            vision_msgs::ObjectHypothesisWithPose hyp;
            hyp.id = 0; hyp.score = 1.0; 
            detection.results.push_back(hyp);
            detection_array.detections.push_back(detection);

            // 마커 생성 (무조건 주황색)
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "cones";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = center_x;
            marker.pose.position.y = center_y;
            marker.pose.position.z = marker_z;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.4; 
            marker.scale.y = 0.4;
            marker.scale.z = marker_h;

            marker.color.r = 0.0f; 
            marker.color.g = 0.0f; 
            marker.color.b = 1.0f; 
            marker.color.a = 0.9f;

            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);
        }

        bbox_pub_.publish(detection_array);
        marker_pub_.publish(marker_array);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_cone");
    GigachaLidarCone node;
    ros::spin();
    return 0;
}