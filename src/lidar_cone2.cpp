#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <cmath>
#include <vector>

// PCL Headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
    
    ros::Publisher cloud_roi_pub_;      // 1. 자른 원본 (흰색)
    ros::Publisher cloud_cluster_pub_;  // 2. [★이게 빠졌었음] 뭉친 놈들 (알록달록)
    ros::Publisher marker_pub_;         // 3. 파란 원통
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
        ROS_INFO("GIGACHA LiDAR Cone (Weed-Killer + Visualizer) Starting...");
        
        // ROI 설정
        nh_.param<float>("roi_min_x", roi_min_x_, 0.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 8.0f); 
        nh_.param<float>("roi_min_y", roi_min_y_, -3.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 3.0f);
        
        // Z축 잡초 제거 설정
        nh_.param<float>("roi_min_z", roi_min_z_, -0.5f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 0.2f); 

        // 클러스터링 설정
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.3f); 
        nh_.param<int>("min_cluster_size", min_cluster_size_, 3); 
        nh_.param<int>("max_cluster_size", max_cluster_size_, 300);

        cloud_roi_pub_     = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_roi", 1);
        
        // [★이게 빠졌었음]
        cloud_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_clustered", 1); 
        
        bbox_pub_          = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        marker_pub_        = nh_.advertise<visualization_msgs::MarkerArray>("/gigacha/lidar/markers", 1);
        
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

        // 1. CropBox
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_origin);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        // ROI 확인용 Publish
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(*cloud_crop, out);
        out.header = msg->header;
        cloud_roi_pub_.publish(out);

        // 점이 너무 적으면 클러스터링 하지 말고 리턴 (안전장치)
        if (cloud_crop->size() < (size_t)min_cluster_size_) return;

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
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        // [★이게 빠졌었음] 클러스터링된 점들을 모아서 보여줄 구름 변수 선언
        pcl::PointCloud<PointT>::Ptr cloud_clustered_visual(new pcl::PointCloud<PointT>);
        cloud_clustered_visual->header = cloud_origin->header;

        int cluster_id = 0;
        float ground_z = -1.0f; // TF 1.0m 기준 바닥

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            
            // 클러스터 점 추출
            for (const auto &idx : indices.indices) {
                PointT p = cloud_crop->points[idx];
                
                // 시각화를 위해 Intensity를 ID로 변경
                // 이렇게 하면 Rviz에서 색깔이 다르게 보임!
                p.intensity = (float)cluster_id + 10.0f; 
                
                cluster->points.push_back(p);
            }

            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float size_x = max_pt[0] - min_pt[0];
            float size_y = max_pt[1] - min_pt[1];
            
            // [필터링]
            if (size_x > 0.5f || size_y > 0.5f) continue;
            if (min_pt[2] > (ground_z + 1.0f)) continue;

            // [★중요] 필터링을 통과한 "진짜 꼬깔 점"들만 시각화 클라우드에 넣음
            for (const auto& p : cluster->points) {
                cloud_clustered_visual->points.push_back(p);
            }

            // 마커 생성 로직 (기존과 동일)
            float center_x = (min_pt[0] + max_pt[0]) / 2.0f;
            float center_y = (min_pt[1] + max_pt[1]) / 2.0f;
            float fixed_height = 0.7f; 
            float marker_center_z = ground_z + (fixed_height / 2.0f);

            vision_msgs::Detection3D detection;
            detection.header = msg->header;
            detection.bbox.center.position.x = center_x;
            detection.bbox.center.position.y = center_y;
            detection.bbox.center.position.z = marker_center_z;
            detection.bbox.center.orientation.w = 1.0;
            detection.bbox.size.x = 0.4f; 
            detection.bbox.size.y = 0.4f;
            detection.bbox.size.z = fixed_height;
            
            vision_msgs::ObjectHypothesisWithPose hyp;
            hyp.id = 0; hyp.score = 1.0; 
            detection.results.push_back(hyp);
            detection_array.detections.push_back(detection);

            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "cones";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detection.bbox.center;
            marker.scale.x = 0.4; 
            marker.scale.y = 0.4;
            marker.scale.z = fixed_height; 

            // 파란색
            marker.color.r = 0.0f; 
            marker.color.g = 0.0f; 
            marker.color.b = 1.0f; 
            marker.color.a = 0.9f;
            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);
        }

        // 3. Publish
        bbox_pub_.publish(detection_array);
        marker_pub_.publish(marker_array);
        
        // [★이게 빠졌었음] 클러스터링 결과 점구름 Publish
        if (!cloud_clustered_visual->empty()) {
            sensor_msgs::PointCloud2 out_cluster;
            pcl::toROSMsg(*cloud_clustered_visual, out_cluster);
            out_cluster.header = msg->header;
            cloud_cluster_pub_.publish(out_cluster);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_cone");
    GigachaLidarCone node;
    ros::spin();
    return 0;
}