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

// 지면제거 patchwork
#include <patchworkpp/patchworkpp.h

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
    ros::Publisher cloud_ground_removed_pub_; // 지면 제거 결과
    ros::Publisher cloud_cluster_pub_;        // 클러스터링 결과
    ros::Publisher bbox_pub_;                 // 박스 결과
    ros::Publisher marker_pub_;               // 시각화 마커

    // Patchwork++ 객체
    boost::shared_ptr<PatchWorkpp::PatchWorkpp> patchwork_ptr_;

    float voxel_size_;
    float roi_min_x_, roi_max_x_;
    float roi_min_y_, roi_max_y_;
    float roi_min_z_, roi_max_z_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

    // Patchwork++ 파라미터
    float sensor_height_;
    int num_iter_;
    int num_lpr_;
    int num_min_pts_;
    float th_seeds_;
    float th_dist_;
    float max_r_;
    float min_r_;
    float uprightness_th_;
    bool verbose_;

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

        // 2. Patchwork++ 파라미터 로드
        // [중요] sensor_height를 실제 차량 라이다 높이로 맞춰야 함!
        nh_.param<float>("sensor_height", sensor_height_, 1.55f); 
        
        nh_.param<int>("num_iter", num_iter_, 3);
        nh_.param<int>("num_lpr", num_lpr_, 20);
        nh_.param<int>("num_min_pts", num_min_pts_, 10);
        nh_.param<float>("th_seeds", th_seeds_, 0.4f);
        nh_.param<float>("th_dist", th_dist_, 0.3f);
        nh_.param<float>("max_r", max_r_, 80.0f);
        nh_.param<float>("min_r", min_r_, 2.7f); // 차량 본체 필터링용 (반경 2.7m 이내 무시)
        nh_.param<float>("uprightness_th", uprightness_th_, 0.707f); 
        nh_.param<bool>("verbose", verbose_, false);

        // 3. Patchwork++ 객체 생성 및 초기화
        PatchWorkpp::Parameters params;
        params.verbose = verbose_;
        params.sensor_height = sensor_height_;
        params.num_iter = num_iter_;
        params.num_lpr = num_lpr_;
        params.num_min_pts = num_min_pts_;
        params.th_seeds = th_seeds_;
        params.th_dist = th_dist_;
        params.max_r = max_r_;
        params.min_r = min_r_;
        params.uprightness_th = uprightness_th_;

        patchwork_ptr_.reset(new Patchworkpp::PatchWorkpp(params));

        // Publisher 설정 (cloud_origin_pub_ 복구)
        cloud_origin_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_origin", 1);
        cloud_ground_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_ground_removed", 1);
        cloud_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_clustered", 1);
        bbox_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/gigacha/lidar/markers", 1);
        
        // 5. Subscriber 설정
        lidar_sub_ = nh_.subscribe("/lidar3D", 1, &GigachaLidarClustering::lidarCallback, this);
        
        ROS_INFO("Initialized Successfully.");
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud_origin);

        if (cloud_origin->empty()) return;

        // --- Step 1: Patchwork++ Ground Removal ---
        // (원본 데이터에서 바로 지면 제거 수행)
        pcl::PointCloud<PointT> cloud_ground, cloud_nonground;
        
        // estimateGround(입력, 지면출력, 비지면출력)
        patchwork_ptr_->estimateGround(*cloud_origin, cloud_ground, cloud_nonground);

        pcl::PointCloud<PointT>::Ptr cloud_obstacles(new pcl::PointCloud<PointT>);
        *cloud_obstacles = cloud_nonground; // 지면이 제거된 포인트들

        // 결과 확인용 Publish
        sensor_msgs::PointCloud2 out_obs;
        pcl::toROSMsg(*cloud_obstacles, out_obs); 
        out_obs.header = msg->header;
        cloud_ground_removed_pub_.publish(out_obs);

        if (cloud_obstacles->empty()) return;

        // --- Step 2: ROI (Crop Box) ---
        // 지면이 없어진 상태에서 관심 영역만 자름
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_obstacles);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        crop_filter.filter(*cloud_crop);

        if (cloud_crop->empty()) return;

        // --- Step 3: Voxel Grid Downsampling ---
        // 클러스터링 속도 향상을 위해 점 개수 줄임
        pcl::PointCloud<PointT>::Ptr cloud_final(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_crop);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*cloud_final);

        if (cloud_final->empty()) return;

        // --- Step 4: Clustering ---
        // Z축 제거 후 2D로 투영하여 클러스터링 (더 정확함)
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

        // 결과 담을 메시지 준비
        vision_msgs::Detection3DArray detection_array;
        detection_array.header = msg->header; 
        
        visualization_msgs::MarkerArray marker_array; 
        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        
        // [수정] 헤더 할당 시 PCL 변환 함수 사용 (에러 해결)
        pcl_conversions::toPCL(msg->header, cloud_clustered->header);

        int cluster_id = 0;
        
        // --- Step 5: Bounding Box Fitting ---
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            // 인덱스는 cloud_final 기준
            for (const auto &idx : indices.indices)
            {
                PointT p = cloud_final->points[idx];
                p.intensity = static_cast<float>(cluster_id % 100); 
                cluster->points.push_back(p);
                cloud_clustered->points.push_back(p);
            }
            
            // L-Shape Fitting
            BoxInfo box = fitLShape(cluster);

            // [필터링 로직]
            float min_side = std::min(box.len, box.wid); 
            float max_side = std::max(box.len, box.wid); 

            // 너무 작거나 너무 큰 노이즈 제거
            if (max_side < 0.2f) continue;  
            if (max_side > 20.0f) continue; 
            if (box.hgt < 0.2f) continue; // 높이 낮은 물체 제거

            // 1. 사람 (작은 박스)
            bool is_person = (min_side < 1.0f) && (max_side < 1.2f);
            
            // 2. 승용차 (일반적인 크기)
            bool is_car = (min_side >= 1.0f && min_side < 2.5f) && (max_side >= 2.0f && max_side < 6.0f);
            
            // 3. 버스/트럭 (크고 도로 위에 있는 것)
            bool is_large = (min_side >= 2.0f && min_side < 3.5f) && (max_side >= 5.5f && max_side < 19.0f);
            bool is_on_road = std::abs(box.y) < 4.5f; // 좌우 4.5m 이내
            bool is_bus = is_large && is_on_road;

            // 셋 다 아니면 스킵
            if (!is_person && !is_car && !is_bus) continue; 

            // Detection3D 메시지 채우기
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

            // 마커 채우기
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "bbox"; 
            marker.id = cluster_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = detection.bbox.center;
            marker.pose.orientation = detection.bbox.center.orientation;
            marker.scale = detection.bbox.size;
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.4f; // 초록색 반투명
            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);

            cluster_id++;
        }

        // Publish
        cloud_origin_pub_.publish(*cloud_origin);
        cloud_cluster_pub_.publish(*cloud_clustered);
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