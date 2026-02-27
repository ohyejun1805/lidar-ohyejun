#include <ros/ros.h>

#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h> 

// MORAI 시뮬레이터 차량 상태 토픽을 받기 위한 헤더 추가
#include <morai_msgs/EgoVehicleStatus.h> 

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
#include <pcl/common/centroid.h>

using PointT = pcl::PointXYZI; 

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

    float best_score = std::numeric_limits<float>::max(); //점수 최소화
    float best_angle = 0.0f;
    
    // 결과 저장용 변수들
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

        //회전 변환된 좌표들 저장해둠.(distance 계산용)
        std::vector<float> rxs,rys;
        rxs.reserve(points_2d.size());
        rys.reserve(points_2d.size());

        for (const auto& p : points_2d) 
        {
            float rx = p.x() * cos_t + p.y() * sin_t;
            float ry = -p.x() * sin_t + p.y() * cos_t;

            rxs.push_back(rx);
            rys.push_back(ry);

            if (rx < current_min_x) current_min_x = rx;
            if (rx > current_max_x) current_max_x = rx;
            if (ry < current_min_y) current_min_y = ry;
            if (ry > current_max_y) current_max_y = ry;
        }

        float width = current_max_x - current_min_x;
        float depth = current_max_y - current_min_y;
        float area = width * depth;

        float dist_sum = 0.0f;

        for (size_t i = 0; i < points_2d.size(); i++)
        {
            // 각 점이 4개의 변 중 "가장 가까운 변"까지의 거리를 구함
            float d_xmin = std::abs(rxs[i] - current_min_x);
            float d_xmax = std::abs(rxs[i] - current_max_x);
            float d_ymin = std::abs(rys[i] - current_min_y);
            float d_ymax = std::abs(rys[i] - current_max_y);

            // X축 변과 Y축 변 중 더 가까운 쪽을 선택 
            float min_d_x = std::min(d_xmin, d_xmax);
            float min_d_y = std::min(d_ymin, d_ymax);
            
            // 최종적으로 가장 가까운 테두리와의 거리
            dist_sum += std::min(min_d_x, min_d_y);
        }

        float score = area + dist_sum;

        if (score < best_score) 
        {
            best_score = score;
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

//확장 칼만필터
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

// tracking
class GigachaLidarTracking
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber detection_sub_;
    ros::Subscriber ego_sub_; 
    
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
        
        bool has_moved;       // 한 번이라도 움직였는지 기록
        bool is_large_size;   // 현재 박스가 대형인지 기록
        double abs_velocity;  // RViz 디버깅용: 계산된 절대 속도 저장
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

    // 인자를 tracks_ 배열 자체를 받도록 변경하여 내부 속도/상태 데이터를 꺼내볼 수 있도록 함
    void publishMarkers(const std_msgs::Header& header, const std::vector<Track>& track_list)
    {
        visualization_msgs::MarkerArray marker_array;
        
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (const auto& track : track_list)
        {
            // 유효하지 않은(사라졌거나 아직 안정화 안 된) 트랙은 그리지 않음
            if (track.disappeared_count > 0 || track.age <= 3) continue;

            const auto& det = track.detection;
            int track_id = track.track_id;

            // ==========================================
            // 1. 박스 (CUBE) 마커 생성
            // ==========================================
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "tracked_objects";
            marker.id = track_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = track.kf.getPosition();
            marker.pose.orientation = det.bbox.center.orientation;
            marker.scale = det.bbox.size;
            
            // 움직임 여부에 따른 색상 로직 (동적: 빨간색, 정적: 초록색)
            if (track.has_moved) {
                marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; 
            } 
            else 
            {
                marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; 
            }
            marker.color.a = 0.5f; 
            marker.lifetime = ros::Duration(0.15); 
            marker_array.markers.push_back(marker);

            // ==========================================
            // 2. 텍스트 정보 (TEXT) 마커 생성
            // ==========================================
            visualization_msgs::Marker text_marker = marker;
            text_marker.ns = "tracked_info"; // 네임스페이스 분리
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            
            // ID, 상태(Move/Stop), 절대속도(m/s) 포맷팅
            char text_buf[128];
            std::snprintf(text_buf, sizeof(text_buf), "ID: %d %s\n%.1f m/s", 
                          track_id, 
                          (track.has_moved ? "[Move]" : "[Stop]"), 
                          track.abs_velocity);
                          
            text_marker.text = text_buf;
            text_marker.scale.z = 1.0; 
            text_marker.color.r = 1.0f; text_marker.color.g = 1.0f; text_marker.color.b = 1.0f; 
            text_marker.color.a = 1.0f;
            text_marker.pose.position.z += (det.bbox.size.z / 2.0 + 0.8); // 박스 위로 조금 더 띄움
            
            marker_array.markers.push_back(text_marker);

            // ==========================================
            // 3. 방향 지시 화살표 (ARROW) 마커 생성
            // ==========================================
            // [추가] EKF 필터가 추적하고 있는 객체의 진행 방향(Yaw)을 화살표로 그림
            if (track.has_moved && track.abs_velocity > 0.5) // 정지된 객체에는 화살표를 안 그림
            {
                visualization_msgs::Marker arrow_marker = marker;
                arrow_marker.ns = "tracked_heading";
                arrow_marker.id = track_id;
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;

                // Scale: x=화살표 대 길이, y=대 두께, z=화살표 머리 두께
                // [수정] std::max 타입 불일치 오류 해결 (2.0f -> 2.0)
                arrow_marker.scale.x = std::max(det.bbox.size.x, 2.0) + 1.0; 
                arrow_marker.scale.y = 0.2; 
                arrow_marker.scale.z = 0.2;

                // 방향(Orientation)은 EKF 필터가 추정한 속도의 방향(state(4))을 사용
                Eigen::Quaterniond q_arrow;
                q_arrow = Eigen::AngleAxisd(track.kf.state_(4), Eigen::Vector3d::UnitZ());
                arrow_marker.pose.orientation.x = q_arrow.x();
                arrow_marker.pose.orientation.y = q_arrow.y();
                arrow_marker.pose.orientation.z = q_arrow.z();
                arrow_marker.pose.orientation.w = q_arrow.w();

                // 투명도 조절 (색상은 박스와 동일하게 Red)
                arrow_marker.color.a = 0.8f; 
                
                marker_array.markers.push_back(arrow_marker);
            }
        }
        
        marker_pub_.publish(marker_array);
    }

    void updateTrack(Track& track, const vision_msgs::Detection3D& detection, double dt)
    {
        geometry_msgs::Point measurement_pos = getCenter(detection);
        
        double measurement_yaw = getYawFromQuaternion(detection.bbox.center.orientation);

        double track_v = track.kf.state_(3);
        double track_yaw = track.kf.state_(4);

        if (std::abs(track_v) > 2.0) 
        {
            double vel_angle = track_yaw; 
            double diff = measurement_yaw - vel_angle;
            
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;

            double abs_diff = std::abs(diff);

            if (abs_diff > (M_PI * 0.75)) 
            { 
                 measurement_yaw += M_PI;
            }
            else if (abs_diff > (M_PI * 0.25) && abs_diff < (M_PI * 0.75)) 
            { 
                 measurement_yaw = vel_angle; 
            }
            
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

        // ----------------------------------------------------------------
        // 2D 벡터를 이용한 완벽한 절대 속도 크기(Magnitude) 계산
        // ----------------------------------------------------------------
        double obj_rel_v = track.kf.state_(3); 
        
        double obj_rel_vx = obj_rel_v * cos(track.kf.state_(4)); 
        double obj_rel_vy = obj_rel_v * sin(track.kf.state_(4)); 

        double abs_vx = obj_rel_vx + my_car_velocity_; 
        double abs_vy = obj_rel_vy;

        double abs_v_mag = std::sqrt(abs_vx * abs_vx + abs_vy * abs_vy);

        // 계산된 절대 속도를 구조체에 저장 (RViz 디버깅용)
        track.abs_velocity = abs_v_mag;

        // 여기서 기존 track.kf.setVelocityZero(); 삭제 완료! 
        // 칼만필터 상태 강제 초기화는 필터 추정을 꼬이게 만들므로 제거해야 합니다.

        // 칼만 필터가 안정화(age > 3)된 후, 절대 속력 크기가 1.5 m/s 이상이면 움직인 것으로 확정!
        if (track.age > 3 && abs_v_mag > 1.5) 
        {
            track.has_moved = true; 
        }

        float max_s = std::max(detection.bbox.size.x, detection.bbox.size.y);
        if (max_s > 5.8f) 
        {
            track.is_large_size = true;
        } 
        else 
        {
            track.is_large_size = false;
        }

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
            
            new_track.has_moved = false; 
            new_track.is_large_size = false;
            new_track.abs_velocity = 0.0; // 초기 속도값 할당

            geometry_msgs::Point init_pos = getCenter(detections.detections[det_idx]);
            double init_yaw = getYawFromQuaternion(detections.detections[det_idx].bbox.center.orientation);
            
            new_track.kf.init(init_pos, init_yaw);
            
            tracks_.push_back(new_track);
        }
    }
    
public:
    GigachaLidarTracking() : nh_("~"), next_track_id_(1), my_car_velocity_(0.0)
    {
        ROS_INFO("GIGACHA LiDAR Tracking Node (Velocity-based Wall Filtering) Starting...");
        
        nh_.param<float>("max_mahalanobis_distance", max_mahalanobis_distance_, 3.0f);
        nh_.param<float>("max_disappeared_frames", max_disappeared_frames_, 5.0f);
        nh_.param<float>("gating_threshold", gating_threshold_, 10.0f); 
        
        detection_sub_ = nh_.subscribe("/gigacha/lidar/bounding_boxes", 1, &GigachaLidarTracking::detectionCallback, this);
        ego_sub_ = nh_.subscribe("/Ego_topic", 1, &GigachaLidarTracking::egoCallback, this);
        
        tracking_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/tracked_objects", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracked_objects", 1);
        
        last_update_time_ = ros::Time::now();
    }
    
    void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
    {
        my_car_velocity_ = msg->velocity.x; 
    }
    
    void detectionCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        if (dt < 0.001 || dt > 1.0) dt = 0.1;
        last_update_time_ = current_time;
        
        vision_msgs::Detection3DArray processed_msg = *msg; 
        std::vector<int> valid_indices; 

        for (size_t i = 0; i < processed_msg.detections.size(); ++i)
        {
            pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(processed_msg.detections[i].source_cloud, *cluster_cloud);

            if(cluster_cloud->empty()) continue;

            BoxInfo box = fitLShape(cluster_cloud);

            float min_side = std::min(box.len, box.wid); 
            float max_side = std::max(box.len, box.wid); 

            float safe_min_side = std::max(min_side, 0.01f); 
            float aspect_ratio = max_side / safe_min_side;

            if (max_side < 0.2f) continue;   
            if (max_side > 20.0f) continue;  
            if (box.hgt < 0.5f) continue;    

            bool is_person = (max_side < 1.0f) && (min_side < 0.8f);
            bool is_car = (min_side >= 0.2f && min_side <= 2.5f) && (max_side >= 1.2f && max_side <= 5.8f);
            bool is_large = (max_side > 5.8f && max_side <= 19.0f);

            if (is_person) {} 
            else if (is_car) 
            {
                if (aspect_ratio > 15.0f) continue; 
            }
            else if (is_large) 
            {
                if (box.hgt < 2.2f || box.hgt > 4.5f) {continue;}
            }
            else {
                continue;
            }

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
            
            // 빈 배열일 때도 tracks_ 기준으로 그려 기존 객체가 사라지는 과정을 RViz에서 확인
            publishMarkers(msg->header, tracks_); 
            return;
        }
        
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
        
        for (const auto& track : tracks_) 
        {
            if (track.disappeared_count == 0 && track.age > 3) 
            {
                bool should_publish = true; 

                if (track.is_large_size) 
                {
                    if (!track.has_moved) {
                        should_publish = false; 
                    }
                }

                if (should_publish) 
                {
                    vision_msgs::Detection3D tracked_det = track.detection;
                    tracked_det.bbox.center.position = track.kf.getPosition();
                    
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
        }
        
        tracking_pub_.publish(tracked_array);
        
        // [수정] 최종 필터링된 tracked_array가 아닌, 모든 트랙 정보가 담긴 tracks_를 넘김
        // 이를 통해 RViz에서는 "필터링되어 무시된 대형 벽(초록색)"과 "살아남은 자동차(빨간색, 화살표)"를 동시에 디버깅 가능합니다.
        publishMarkers(msg->header, tracks_); 
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gigacha_lidar_tracking");
    GigachaLidarTracking node;
    ros::spin();
    return 0;
}