#include <ros/ros.h>

#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h> 

#include <limits>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

// PCL 라이브러리 (L-shape fitting)
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using PointT = pcl::PointXYZI; // 필요에 따라 PointXYZ로 변경 가능

struct BoxInfo 
{
    float x, y, z;       
    float len, wid, hgt; 
    float yaw;           
};

// Search-Based L-Shape Fitting
BoxInfo fitLShape(const pcl::PointCloud<PointT>::Ptr& cluster)
{
    BoxInfo box;
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);//x,y,z축 별로 minmax 찾음.

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

//확장 칼만만
class ExtendedKalmanFilter
{
private:
    public: Eigen::VectorXd state_; 
    private:
    Eigen::MatrixXd P_; 
    Eigen::MatrixXd Q_; 
    Eigen::MatrixXd R_; 
    bool initialized_;

public:
    ExtendedKalmanFilter() : initialized_(false)
    {
        state_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6);
        
        P_(0,0) = 1.0; 
        P_(1,1) = 1.0; 
        P_(2,2) = 1.0;
        P_(3,3) = 100.0; 
        P_(4,4) = 100.0; 
        P_(5,5) = 100.0;

        Q_ = Eigen::MatrixXd::Identity(6, 6);
        Q_ *= 0.1; 
        Q_(3,3) = 2.0; 
        Q_(4,4) = 0.1; 
        Q_(5,5) = 0.5; 

        R_ = Eigen::MatrixXd::Identity(4, 4);
        R_(0,0) = 0.1; 
        R_(1,1) = 0.1; 
        R_(2,2) = 0.1;
        R_(3,3) = 0.5; 
    }

    void init(const geometry_msgs::Point& pos, double yaw)
    {
        state_ << pos.x, pos.y, pos.z, 0, yaw, 0;
        initialized_ = true;
    }

    void setVelocityZero() 
    {
        state_(3) = 0.0; 
        state_(5) = 0.0; 
    }

    void predict(double dt)
    {
        if (!initialized_) return;

        double v = state_(3);
        double yaw = state_(4);
        double yaw_rate = state_(5);

        if (fabs(yaw_rate) > 0.001) 
        {
            state_(0) += (v / yaw_rate) * (sin(yaw + yaw_rate * dt) - sin(yaw));
            state_(1) += (v / yaw_rate) * (-cos(yaw + yaw_rate * dt) + cos(yaw));
        } else 
        {
            state_(0) += v * cos(yaw) * dt;
            state_(1) += v * sin(yaw) * dt;
        }
        state_(4) += yaw_rate * dt;

        while (state_(4) > M_PI) state_(4) -= 2.0 * M_PI;
        while (state_(4) < -M_PI) state_(4) += 2.0 * M_PI;

        Eigen::MatrixXd Fj = Eigen::MatrixXd::Identity(6, 6);
        if (fabs(yaw_rate) > 0.001) 
        {
            double theta_new = yaw + yaw_rate * dt;
            Fj(0, 3) = (1/yaw_rate) * (sin(theta_new) - sin(yaw));
            Fj(0, 4) = (v/yaw_rate) * (cos(theta_new) - cos(yaw));
            Fj(0, 5) = (dt*v/yaw_rate)*cos(theta_new) - (v/pow(yaw_rate,2))*(sin(theta_new)-sin(yaw));
            Fj(1, 3) = (1/yaw_rate) * (-cos(theta_new) + cos(yaw));
            Fj(1, 4) = (v/yaw_rate) * (sin(theta_new) - sin(yaw));
            Fj(1, 5) = (dt*v/yaw_rate)*sin(theta_new) - (v/pow(yaw_rate,2))*(-cos(theta_new)+cos(yaw));
        } 
        else 
        {
            Fj(0, 3) = cos(yaw) * dt;
            Fj(0, 4) = -v * sin(yaw) * dt;
            Fj(1, 3) = sin(yaw) * dt;
            Fj(1, 4) = v * cos(yaw) * dt;
        }
        Fj(4, 5) = dt;

        P_ = Fj * P_ * Fj.transpose() + Q_;
    }

    void update(const geometry_msgs::Point& pos, double meas_yaw)
    {
        if (!initialized_) {
            init(pos, meas_yaw);
            return;
        }

        Eigen::VectorXd z(4);
        z << pos.x, pos.y, pos.z, meas_yaw;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 6);
        H(0,0) = 1.0; 
        H(1,1) = 1.0; 
        H(2,2) = 1.0; 
        H(3,4) = 1.0; 

        Eigen::VectorXd y = z - H * state_;

        //각도 정규화화
        while (y(3) > M_PI) y(3) -= 2.0 * M_PI;
        while (y(3) < -M_PI) y(3) += 2.0 * M_PI;

        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        state_ = state_ + (K * y);
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
        
        while (state_(4) > M_PI) state_(4) -= 2.0 * M_PI;
        while (state_(4) < -M_PI) state_(4) += 2.0 * M_PI;
    }

    geometry_msgs::Point getPosition() const 
    {
        geometry_msgs::Point p;
        p.x = state_(0); p.y = state_(1); p.z = state_(2);
        return p;
    }

    Eigen::MatrixXd getCovariance() const { return P_; }
    bool isInitialized() const { return initialized_; }
};

//헝가리안
class HungarianAlgorithm
{
public:
    static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix)
    {
        if (cost_matrix.empty() || cost_matrix[0].empty())
            return std::vector<int>(cost_matrix.size(), -1);
        
        int n = cost_matrix.size();
        int m = cost_matrix[0].size();
        int size = std::max(n, m);
        std::vector<std::vector<double>> matrix(size, std::vector<double>(size, 0.0));
        
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                matrix[i][j] = cost_matrix[i][j];
        
        std::vector<int> assignment = munkres(matrix);
        std::vector<int> result(n, -1);
        for (int i = 0; i < n && i < size; i++)
            if (assignment[i] < m) result[i] = assignment[i];
        
        return result;
    }

private:
    static std::vector<int> munkres(std::vector<std::vector<double>>& matrix)
    {
        int n = matrix.size();
        for (int i = 0; i < n; i++) {
            double min_val = *std::min_element(matrix[i].begin(), matrix[i].end());
            for (int j = 0; j < n; j++) matrix[i][j] -= min_val;
        }
        for (int j = 0; j < n; j++) {
            double min_val = matrix[0][j];
            for (int i = 1; i < n; i++) min_val = std::min(min_val, matrix[i][j]);
            for (int i = 0; i < n; i++) matrix[i][j] -= min_val;
        }

        std::vector<int> assignment(n, -1);
        std::vector<bool> row_covered(n, false);
        std::vector<bool> col_covered(n, false);

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (matrix[i][j] == 0.0 && !row_covered[i] && !col_covered[j]) {
                    assignment[i] = j;
                    row_covered[i] = true;
                    col_covered[j] = true;
                    break;
                }
            }
        }
        return assignment;
    }
};

// ------------------------------------------------------------
// [Class] Main Tracking Node
// ------------------------------------------------------------
class GigachaLidarTracking
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber detection_sub_;
    ros::Subscriber odom_sub_; 
    
    ros::Publisher tracking_pub_;
    ros::Publisher marker_pub_;
    
    float max_mahalanobis_distance_;
    float max_disappeared_frames_;
    float gating_threshold_;
    
    double my_car_velocity_; 

    struct Track
    {
        int track_id;
        ExtendedKalmanFilter kf; 
        vision_msgs::Detection3D detection;
        int disappeared_count;
        ros::Time last_seen;
        int age;
    };
    
    std::vector<Track> tracks_;
    int next_track_id_;
    ros::Time last_update_time_;

    double getYawFromQuaternion(const geometry_msgs::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    geometry_msgs::Point getCenter(const vision_msgs::Detection3D& det)
    { 
        return det.bbox.center.position;
    }
    
    double calculateMahalanobisDistance(const geometry_msgs::Point& predicted,
                                        const geometry_msgs::Point& measurement,
                                        const Eigen::MatrixXd& covariance)
    {
        Eigen::Vector3d pred(predicted.x, predicted.y, predicted.z);
        Eigen::Vector3d meas(measurement.x, measurement.y, measurement.z);
        Eigen::Vector3d diff = meas - pred;
        
        Eigen::Matrix3d S = covariance.block<3, 3>(0, 0);
        if(S.determinant() < 1e-6) return 100.0;
        
        Eigen::Matrix3d S_inv = S.inverse();
        return std::sqrt(diff.transpose() * S_inv * diff);
    }
    
    std::vector<std::vector<double>> buildCostMatrix(const vision_msgs::Detection3DArray& detections)
    {
        std::vector<std::vector<double>> cost_matrix;
        if (tracks_.empty() || detections.detections.empty()) return cost_matrix;
        
        cost_matrix.resize(tracks_.size());
        for (size_t i = 0; i < tracks_.size(); i++)
        {
            cost_matrix[i].resize(detections.detections.size());
            geometry_msgs::Point predicted_pos = tracks_[i].kf.getPosition();
            Eigen::MatrixXd covariance = tracks_[i].kf.getCovariance();
            
            for (size_t j = 0; j < detections.detections.size(); j++)
            {
                geometry_msgs::Point det_center = getCenter(detections.detections[j]);
                double mahal_dist = calculateMahalanobisDistance(predicted_pos, det_center, covariance);
                
                if (mahal_dist > gating_threshold_) cost_matrix[i][j] = 1e6;
                else cost_matrix[i][j] = mahal_dist;
            }
        }
        return cost_matrix;
    }
    
    void associateDetectionsToTracks(const vision_msgs::Detection3DArray& detections,
                                     std::vector<int>& matched_tracks,
                                     std::vector<int>& unmatched_detections)
    {
        matched_tracks.clear();
        unmatched_detections.clear();
        
        if (tracks_.empty()) {
            for (size_t i = 0; i < detections.detections.size(); i++) unmatched_detections.push_back(i);
            return;
        }
        if (detections.detections.empty()) {
            matched_tracks.resize(tracks_.size(), -1);
            return;
        }
        
        std::vector<std::vector<double>> cost_matrix = buildCostMatrix(detections);
        matched_tracks = HungarianAlgorithm::solve(cost_matrix);
        
        std::vector<bool> det_matched(detections.detections.size(), false);
        for (size_t i = 0; i < matched_tracks.size(); i++) {
            if (matched_tracks[i] >= 0 && matched_tracks[i] < (int)detections.detections.size()) {
                if (cost_matrix[i][matched_tracks[i]] > max_mahalanobis_distance_) matched_tracks[i] = -1;
                else det_matched[matched_tracks[i]] = true;
            }
        }
        
        for (size_t j = 0; j < detections.detections.size(); j++)
            if (!det_matched[j]) unmatched_detections.push_back(j);
    }

    void publishMarkers(const vision_msgs::Detection3DArray& tracked_array)
    {
        visualization_msgs::MarkerArray marker_array;
        
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (size_t i = 0; i < tracked_array.detections.size(); ++i)
        {
            const auto& det = tracked_array.detections[i];
            int track_id = (det.results.empty() ? -1 : det.results[0].id);
            if (track_id == -1) continue;

            visualization_msgs::Marker marker;
            marker.header = tracked_array.header;
            marker.ns = "tracked_objects";
            marker.id = track_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = det.bbox.center;
            // 박스 마커 그릴 때 현재 L-Shape Orientation 반영
            marker.pose.orientation = det.bbox.center.orientation;
            marker.scale = det.bbox.size;
            
            marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; 
            marker.color.a = 0.5f; 
            
            marker.lifetime = ros::Duration(0.15); 
            marker_array.markers.push_back(marker);

            visualization_msgs::Marker text_marker = marker;
            text_marker.ns = "tracked_id";
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.text = "ID: " + std::to_string(track_id); 
            text_marker.scale.z = 1.0; 
            text_marker.color.r = 1.0f; text_marker.color.g = 1.0f; text_marker.color.b = 1.0f; 
            text_marker.color.a = 1.0f;
            text_marker.pose.position.z += 1.0; 
            
            marker_array.markers.push_back(text_marker);
        }
        
        marker_pub_.publish(marker_array);
    }

    void updateTrack(Track& track, const vision_msgs::Detection3D& detection, double dt)
    {
        geometry_msgs::Point measurement_pos = getCenter(detection);
        
        // detection은 detectionCallback에서 L-Shape Fitting을 거쳐 Heading이 계산된 상태임
        double measurement_yaw = getYawFromQuaternion(detection.bbox.center.orientation);

        double track_v = track.kf.state_(3);
        double track_yaw = track.kf.state_(4);

        // 속도 벡터를 이용한 Heading 보정
        // 속도가 충분히 빠르면(2.0 m/s 이상), 형상(L-Shape)보다 이동 방향(Velocity Vector)을 믿음
        if (std::abs(track_v) > 2.0) 
        {
            double vel_angle = track_yaw; 
            double diff = measurement_yaw - vel_angle;
            
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;

            double abs_diff = std::abs(diff);

            // 180도 반대 (차는 앞으로 가는데 박스 헤딩은 뒤를 봄)
            if (abs_diff > (M_PI * 0.75)) 
            { 
                 measurement_yaw += M_PI;
            }
            // 90도 직각 (정사각형 차량 등, 옆으로 누운 경우)
            else if (abs_diff > (M_PI * 0.25) && abs_diff < (M_PI * 0.75)) 
            { 
                 // 이동 방향으로 강제 동기화
                 measurement_yaw = vel_angle; 
            }
            
            // 다시 정규화
            while (measurement_yaw > M_PI) measurement_yaw -= 2.0 * M_PI;
            while (measurement_yaw < -M_PI) measurement_yaw += 2.0 * M_PI;
        }

        if (track.age < 3) 
        {
            double last_yaw = track.kf.state_(4);
            double diff_yaw = measurement_yaw - last_yaw;

            while (diff_yaw > M_PI) diff_yaw -= 2.0 * M_PI;
            while (diff_yaw < -M_PI) diff_yaw += 2.0 * M_PI;

            track.kf.state_(5) = diff_yaw / dt; 
        }

        track.kf.predict(dt);
        track.kf.update(measurement_pos, measurement_yaw);

        double obj_rel_v = track.kf.state_(3); 
        double obj_vx = obj_rel_v * cos(track.kf.state_(4)); 
        double abs_vx = obj_vx + my_car_velocity_; 

        if (std::abs(abs_vx) < 1.0) 
        {
            track.kf.setVelocityZero(); 
        }

        // 업데이트된 정보 저장
        track.detection = detection;
        track.disappeared_count = 0;
        track.last_seen = ros::Time::now();
        track.age++;
    }
    
    void updateDisappearedTracks(double dt)
    {
        for (auto& track : tracks_)
            if (track.disappeared_count >= 0) track.kf.predict(dt);
    }
    
    void removeOldTracks()
    {
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const Track& track) {
                return track.disappeared_count > max_disappeared_frames_;
            }), tracks_.end());
    }
    
    void createNewTracks(const vision_msgs::Detection3DArray& detections,
                         const std::vector<int>& unmatched_detections)
    {
        for (int det_idx : unmatched_detections)
        {
            Track new_track;
            new_track.track_id = next_track_id_++;
            new_track.detection = detections.detections[det_idx];
            new_track.disappeared_count = 0;
            new_track.last_seen = ros::Time::now();
            new_track.age = 1;
            
            geometry_msgs::Point init_pos = getCenter(detections.detections[det_idx]);
            // 여기서 detection은 L-Shape로 구해진 Heading을 가짐
            double init_yaw = getYawFromQuaternion(detections.detections[det_idx].bbox.center.orientation);
            
            new_track.kf.init(init_pos, init_yaw);
            
            tracks_.push_back(new_track);
        }
    }
    
public:
    GigachaLidarTracking() : nh_("~"), next_track_id_(1), my_car_velocity_(0.0)
    {
        ROS_INFO("GIGACHA LiDAR Tracking Node (L-Shape + Heading Correction) Starting...");
        
        nh_.param<float>("max_mahalanobis_distance", max_mahalanobis_distance_, 3.0f);
        nh_.param<float>("max_disappeared_frames", max_disappeared_frames_, 5.0f);
        nh_.param<float>("gating_threshold", gating_threshold_, 10.0f); 
        
        detection_sub_ = nh_.subscribe("/gigacha/lidar/bounding_boxes", 1, &GigachaLidarTracking::detectionCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &GigachaLidarTracking::odomCallback, this);
        
        tracking_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/tracked_objects", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracked_objects", 1);
        
        last_update_time_ = ros::Time::now();
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        my_car_velocity_ = msg->twist.twist.linear.x; 
    }
    
    // Detection 콜백에서 매칭 전 L-Shape 계산 수행
    void detectionCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        if (dt < 0.001 || dt > 1.0) dt = 0.1;
        last_update_time_ = current_time;
        
        // 전처리: 수신된 PointCloud로 L-Shape Fitting 수행
        vision_msgs::Detection3DArray processed_msg = *msg; // 수정 위해 복사
        std::vector<int> valid_indices; // 크기 필터링 후 유효한 인덱스

        for (size_t i = 0; i < processed_msg.detections.size(); ++i)
        {
            // 1. PointCloud 복원 (Detection 메시지 안의 source_cloud 필드 사용)
            pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(processed_msg.detections[i].source_cloud, *cluster_cloud);

            if(cluster_cloud->empty()) continue;

            // 2. L-Shape Fitting 수행 (헤딩과 정교한 박스 획득)
            BoxInfo box = fitLShape(cluster_cloud);

            // 3. 필터링 (너무 작거나 큰 박스 제거)
            float min_side = std::min(box.len, box.wid); 
            float max_side = std::max(box.len, box.wid); 

            // 1. 기본 노이즈 제거 (너무 작거나 너무 높거나)
            if (max_side < 0.2f) continue;   // 점 몇 개 뭉친 노이즈
            if (max_side > 20.0f) continue;  // 20m 넘는 건 건물 벽일 확률 높음
            if (box.hgt < 0.5f) continue;    // 높이 50cm 미만(화단, 낮은 턱) 무시

            // 가드레일은 거르려고, 가드레일은 비율이 크니까까
            float aspect_ratio = max_side / (min_side + 1e-4); // 쓰레기값 넣어서 0 들어가는거 방지지

            bool is_on_road = std::abs(box.y) < 5.0f; 

            // 사람
            bool is_person = (max_side < 1.0f) && (min_side < 0.8f);

            // 승용차
            bool is_car = (min_side >= 0.2f && min_side <= 2.5f) && 
                        (max_side >= 1.2f && max_side <= 5.8f);

            // 대형차차
            bool is_large = (max_side > 5.8f && max_side <= 19.0f) && (min_side >= 0.5f);


            if (is_person) {} //사람은 onroad 아니여서 걍 pass
            else if (is_car) {
                if (aspect_ratio > 15.0f && !is_on_road) continue; 
            }
            else if (is_large) {
                if (!is_on_road) continue;
            }
            else {
                continue;
            }

            // Fitting 결과로 Detection 정보 갱신
            // Tracking 알고리즘은 이 값을 보고 매칭을 수행하게 됨
            processed_msg.detections[i].bbox.center.position.x = box.x;
            processed_msg.detections[i].bbox.center.position.y = box.y;
            processed_msg.detections[i].bbox.center.position.z = box.z;

            Eigen::Quaternionf q;
            q = Eigen::AngleAxisf(box.yaw, Eigen::Vector3f::UnitZ());
            processed_msg.detections[i].bbox.center.orientation.x = q.x();
            processed_msg.detections[i].bbox.center.orientation.y = q.y();
            processed_msg.detections[i].bbox.center.orientation.z = q.z();
            processed_msg.detections[i].bbox.center.orientation.w = q.w();

            processed_msg.detections[i].bbox.size.x = box.len;
            processed_msg.detections[i].bbox.size.y = box.wid;
            processed_msg.detections[i].bbox.size.z = box.hgt;

            valid_indices.push_back(i);
        }

        // 유효한 detection만 남긴 새로운 Array 생성
        vision_msgs::Detection3DArray filtered_msg;
        filtered_msg.header = msg->header;
        for(int idx : valid_indices) {
            filtered_msg.detections.push_back(processed_msg.detections[idx]);
        }

        if (filtered_msg.detections.empty()) {
            updateDisappearedTracks(dt);
            removeOldTracks();
            vision_msgs::Detection3DArray empty;
            empty.header = msg->header;
            tracking_pub_.publish(empty);
            publishMarkers(empty);
            return;
        }
        
        // --- 이후 기존 Tracking 로직 (filtered_msg 사용) ---
        
        std::vector<int> matched_tracks, unmatched_detections;
        associateDetectionsToTracks(filtered_msg, matched_tracks, unmatched_detections);
        
        for (size_t i = 0; i < tracks_.size(); i++) {
            if (matched_tracks[i] >= 0 && matched_tracks[i] < (int)filtered_msg.detections.size()) {
                updateTrack(tracks_[i], filtered_msg.detections[matched_tracks[i]], dt);
            } else {
                tracks_[i].disappeared_count++;
                tracks_[i].kf.predict(dt);
            }
        }
        
        createNewTracks(filtered_msg, unmatched_detections);
        removeOldTracks();
        
        vision_msgs::Detection3DArray tracked_array;
        tracked_array.header = msg->header;
        for (const auto& track : tracks_) {
            if (track.disappeared_count == 0 && track.age > 3) {
                vision_msgs::Detection3D tracked_det = track.detection;
                tracked_det.bbox.center.position = track.kf.getPosition();
                
                // Visualization용 마커에는 EKF 위치와 원래 Box 크기를, ID는 KF ID를 사용
                if (tracked_det.results.empty()) {
                    vision_msgs::ObjectHypothesisWithPose hyp;
                    hyp.id = track.track_id; hyp.score = 1.0;
                    tracked_det.results.push_back(hyp);
                } else {
                    tracked_det.results[0].id = track.track_id;
                }
                tracked_array.detections.push_back(tracked_det);
            }
        }
        tracking_pub_.publish(tracked_array);
        publishMarkers(tracked_array);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_tracking");
    GigachaLidarTracking node;
    ros::spin();
    return 0;
}