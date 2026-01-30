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

// [NEW] 트래킹을 위한 꼬깔 구조체
struct TrackedCone {
    int id;
    float x, y, z;
    int life; // 생명력 (몇 프레임동안 살아남을지)
};

class GigachaLidarCone
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    
    ros::Publisher cloud_roi_pub_; 
    ros::Publisher cloud_cluster_pub_;
    ros::Publisher marker_pub_;    
    ros::Publisher bbox_pub_;      

    float roi_min_x_, roi_max_x_;
    float roi_min_y_, roi_max_y_;
    float roi_min_z_, roi_max_z_;
    
    int min_cluster_size_;
    int max_cluster_size_;
    float cluster_tolerance_;

    // [NEW] 트래킹 변수들
    std::vector<TrackedCone> map_cones_; // 기억하고 있는 꼬깔들
    int global_id_counter_;              // 고유 ID 부여용
    int cone_life_time_;                 // 꼬깔의 수명 (프레임 단위)

public:
    GigachaLidarCone() : nh_("~"), global_id_counter_(0)
    {
        ROS_INFO("GIGACHA LiDAR Cone (Weed-Killer + Memory Tracking) Starting...");
        
        nh_.param<float>("roi_min_x", roi_min_x_, 0.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 8.0f); 
        nh_.param<float>("roi_min_y", roi_min_y_, -3.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 3.0f);
        
        nh_.param<float>("roi_min_z", roi_min_z_, -0.6f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 0.2f); 

        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.3f); 
        
        // [★솔루션 1] 점 개수 제한을 1로 낮춤 (미세한 점도 다 잡음)
        nh_.param<int>("min_cluster_size", min_cluster_size_, 3); 
        nh_.param<int>("max_cluster_size", max_cluster_size_, 300);

        // [★솔루션 2] 트래킹 수명 설정 (10프레임 = 약 1초 동안 기억함)
        nh_.param<int>("cone_life_time", cone_life_time_, 10);

        cloud_roi_pub_     = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_roi", 1);
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

        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(*cloud_crop, out);
        out.header = msg->header;
        cloud_roi_pub_.publish(out);

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

        // 이번 프레임에서 발견된 꼬깔 후보들 (Raw Detections)
        std::vector<TrackedCone> current_cones;
        float ground_z = -1.0f; // TF 1.0m 기준 바닥

        pcl::PointCloud<PointT>::Ptr cloud_clustered_visual(new pcl::PointCloud<PointT>);
        cloud_clustered_visual->header = cloud_origin->header;
        int temp_id = 0;

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            for (const auto &idx : indices.indices) {
                PointT p = cloud_crop->points[idx];
                p.intensity = (float)temp_id + 10.0f; 
                cluster->points.push_back(p);
            }
            temp_id++;

            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float size_x = max_pt[0] - min_pt[0];
            float size_y = max_pt[1] - min_pt[1];
            
            // [필터링]
            if (size_x > 0.5f || size_y > 0.5f) continue;
            if (min_pt[2] > (ground_z + 1.0f)) continue;

            // 시각화용 점구름 담기
            for (const auto& p : cluster->points) cloud_clustered_visual->points.push_back(p);

            float center_x = (min_pt[0] + max_pt[0]) / 2.0f;
            float center_y = (min_pt[1] + max_pt[1]) / 2.0f;
            float fixed_height = 0.7f; 
            float marker_center_z = ground_z + (fixed_height / 2.0f);

            // 후보군에 추가
            TrackedCone cone;
            cone.x = center_x;
            cone.y = center_y;
            cone.z = marker_center_z;
            cone.life = cone_life_time_; // 초기 생명력 부여
            cone.id = -1; // 아직 ID 없음
            current_cones.push_back(cone);
        }

        // =========================================================
        // [★트래킹 로직] 기존 꼬깔(Memory)과 새 꼬깔(Current) 매칭
        // =========================================================
        
        for (auto &curr : current_cones)
        {
            int match_idx = -1;
            float min_dist = 0.5f; // 50cm 이내면 같은 꼬깔로 간주

            for (size_t i = 0; i < map_cones_.size(); i++)
            {
                float dx = curr.x - map_cones_[i].x;
                float dy = curr.y - map_cones_[i].y;
                float dist = std::sqrt(dx*dx + dy*dy);

                if (dist < min_dist) {
                    min_dist = dist;
                    match_idx = i;
                }
            }

            if (match_idx != -1) {
                // 매칭 성공: 위치 업데이트 & 생명력 충전 & ID 유지
                map_cones_[match_idx].x = curr.x;
                map_cones_[match_idx].y = curr.y;
                map_cones_[match_idx].life = cone_life_time_; // 생명 연장!
            }
            else {
                // 매칭 실패: 새로운 꼬깔 등장!
                curr.id = global_id_counter_++;
                map_cones_.push_back(curr);
            }
        }

        // 수명 감소 및 사망 처리
        for (auto it = map_cones_.begin(); it != map_cones_.end(); )
        {
            it->life--; // 시간 경과
            if (it->life <= 0) {
                it = map_cones_.erase(it); // 사망 (삭제)
            } else {
                ++it;
            }
        }

        // =========================================================
        // [Publish] 이제는 'Raw 데이터'가 아니라 'Memory(Map)'를 쏘아줌
        // =========================================================

        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header;
        
        visualization_msgs::MarkerArray marker_array; 
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        float fixed_height = 0.7f;

        for (const auto &cone : map_cones_)
        {
            // Detection 메시지
            vision_msgs::Detection3D detection;
            detection.header = msg->header;
            detection.bbox.center.position.x = cone.x;
            detection.bbox.center.position.y = cone.y;
            detection.bbox.center.position.z = cone.z;
            detection.bbox.center.orientation.w = 1.0;
            detection.bbox.size.x = 0.4f; 
            detection.bbox.size.y = 0.4f;
            detection.bbox.size.z = fixed_height;
            
            vision_msgs::ObjectHypothesisWithPose hyp;
            hyp.id = cone.id; // 추적된 ID
            hyp.score = 1.0; 
            detection.results.push_back(hyp);
            detection_array.detections.push_back(detection);

            // 마커 생성
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "cones";
            marker.id = cone.id; // 마커 ID도 고정됨 (깜빡임 X)
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detection.bbox.center;
            
            marker.scale.x = 0.4; 
            marker.scale.y = 0.4;
            marker.scale.z = fixed_height; 

            marker.color.r = 0.0f; 
            marker.color.g = 0.0f; 
            marker.color.b = 1.0f; // 파란색
            marker.color.a = 0.9f;

            // [중요] 생명력이 얼마 안 남았으면(깜빡일 때) 투명하게? 아니, 그냥 진하게!
            // 여기서는 트래킹 믿고 무조건 0.1초 더 살려둠
            marker.lifetime = ros::Duration(0.1); 
            marker_array.markers.push_back(marker);
            
            // ID 텍스트 띄우기 (확인용)
            visualization_msgs::Marker text;
            text.header = msg->header;
            text.ns = "ids";
            text.id = cone.id;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = cone.x;
            text.pose.position.y = cone.y;
            text.pose.position.z = cone.z + 0.5f;
            text.scale.z = 0.3;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.text = "ID: " + std::to_string(cone.id);
            text.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(text);
        }

        bbox_pub_.publish(detection_array);
        marker_pub_.publish(marker_array);
        
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