#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <cmath>
#include <vector>

// PCL
#include <pcl_conversions/pcl_conversions.h> //ROS, PCL 변환 ex)fromROSMsg
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h> // getMinMax3D 등 사용

// Vision Msgs
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

// 공통
#include <pcl/common/transforms.h>

// 마커
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// [Patchwork++] 헤더
#include <patchworkpp/patchworkpp.hpp> 

using PointT = pcl::PointXYZI;

class GigachaLidarClustering
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    
    ros::Publisher cloud_origin_pub_;     
    ros::Publisher cloud_ground_removed_pub_; 
    ros::Publisher cloud_cluster_pub_;        
    ros::Publisher bbox_pub_;         

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
        ROS_INFO("GIGACHA LiDAR Clustering (Point Cloud Packing Mode) Init...");
        
        nh_.param<float>("voxel_size", voxel_size_, 0.15f);
        nh_.param<float>("roi_min_x", roi_min_x_, -20.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 30.0f);
        nh_.param<float>("roi_min_y", roi_min_y_, -7.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 7.0f);
        nh_.param<float>("roi_min_z", roi_min_z_, -1.7f); 
        nh_.param<float>("roi_max_z", roi_max_z_, 2.5f);
        
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.35f);
        nh_.param<int>("min_cluster_size", min_cluster_size_, 5);
        nh_.param<int>("max_cluster_size", max_cluster_size_, 5000);

        patchwork_ptr_.reset(new PatchWorkpp<PointT>(&nh_));

        cloud_origin_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_origin", 1);
        cloud_ground_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_ground_removed", 1);
        cloud_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_clustered", 1);
        bbox_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        
        lidar_sub_ = nh_.subscribe("/lidar3D", 1, &GigachaLidarClustering::lidarCallback, this);
        
        ROS_INFO("Initialized Successfully.");
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud_origin);//ros 메세지 cloud_origin에 저장장

        if (cloud_origin->empty()) return;

        // patchwork++
        pcl::PointCloud<PointT> cloud_ground, cloud_nonground;
        double time_taken;
        patchwork_ptr_->estimate_ground(*cloud_origin, cloud_ground, cloud_nonground, time_taken);

        pcl::PointCloud<PointT>::Ptr cloud_obstacles(new pcl::PointCloud<PointT>);
        *cloud_obstacles = cloud_nonground; 

        sensor_msgs::PointCloud2 out_obs;
        pcl::toROSMsg(*cloud_obstacles, out_obs); 
        out_obs.header = msg->header;
        cloud_ground_removed_pub_.publish(out_obs);

        if (cloud_obstacles->empty()) return;

        // crop
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_obstacles);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        if (cloud_crop->empty()) return;

        // --- Step 3: Voxel Grid Downsampling ---
        pcl::PointCloud<PointT>::Ptr cloud_final(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_crop);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
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

        // --- Output Message Prep ---
        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header; 
        
        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        pcl_conversions::toPCL(msg->header, cloud_clustered->header);

        int cluster_id = 0;
        
        // Packing Data
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
            
            vision_msgs::Detection3D detection;
            detection.header = detection_array.header;

            // 클러스터 포인트 클라우드를 메시지 필드에 담음
            // Tracking 노드에서 이걸 꺼내서 L-Shape Fitting을 수행함
            pcl::toROSMsg(*cluster, detection.source_cloud); 

            // 여기서는 박스 Fitting을 하지 않으므로 대략적인 중심점만 계산
            PointT min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            
            detection.bbox.center.position.x = (min_pt.x + max_pt.x) / 2.0;
            detection.bbox.center.position.y = (min_pt.y + max_pt.y) / 2.0;
            detection.bbox.center.position.z = (min_pt.z + max_pt.z) / 2.0;
            
            // 크기와 방향은 Tracking 노드에서 계산하라고 0 또는 기본값으로 둠
            detection.bbox.size.x = 0.0; 
            detection.bbox.size.y = 0.0; 
            detection.bbox.size.z = 0.0;
            detection.bbox.center.orientation.w = 1.0; 

            vision_msgs::ObjectHypothesisWithPose hypothesis;
            hypothesis.id = 0; 
            hypothesis.score = 1.0; 
            detection.results.push_back(hypothesis);

            detection_array.detections.push_back(detection);
            cluster_id++;
        }

        cloud_cluster_pub_.publish(*cloud_clustered);
        bbox_pub_.publish(detection_array);
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