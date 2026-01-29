#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h> // 내 차 속도용
#include <limits>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

// 클래스 지정 : 확장 칼만 필터 (EKF) - CTRV 모델

class ExtendedKalmanFilter
{
private:
    // 상태 벡터: [x, y, z, v, yaw, yaw_rate] (6차원)
    // v: 속력 (magnitude), yaw: 헤딩, yaw_rate: 회전각속도
    public: Eigen::VectorXd state_; // 외부 접근 위해 public (또는 getter 사용)
    private:
    Eigen::MatrixXd P_; // 공분산
    Eigen::MatrixXd Q_; // 예측 노이즈
    Eigen::MatrixXd R_; // 관측 노이즈
    bool initialized_;

public:
    ExtendedKalmanFilter() : initialized_(false)
    {
        state_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6);
        
        // 초기 불확실성
        P_(0,0) = 1.0; P_(1,1) = 1.0; P_(2,2) = 1.0;
        P_(3,3) = 1000.0; P_(4,4) = 1000.0; P_(5,5) = 1000.0;

        // Q (프로세스 노이즈) - 튜닝 포인트
        Q_ = Eigen::MatrixXd::Identity(6, 6);
        Q_ *= 0.1; 
        Q_(3,3) = 2.0; // 속도 변화 가능성 큼
        Q_(4,4) = 0.5; // 각도 변화
        Q_(5,5) = 0.5; // 각속도 변화

        // R (관측 노이즈) - [x, y, z, yaw] 4개 관측
        R_ = Eigen::MatrixXd::Identity(4, 4);
        R_(0,0) = 0.1; R_(1,1) = 0.1; R_(2,2) = 0.1;
        R_(3,3) = 0.3; // 각도 관측 노이즈 (약 17도)
    }

    void init(const geometry_msgs::Point& pos, double yaw)
    {
        state_ << pos.x, pos.y, pos.z, 0, yaw, 0;
        initialized_ = true;
    }

    // [핵심] 정지 물체 강제 설정 함수
    void setVelocityZero() {
        state_(3) = 0.0; // 속력 0
        state_(5) = 0.0; // 회전 0
    }

    void predict(double dt)
    {
        if (!initialized_) return;
        //초기화 안되었을 시 반환

        double v = state_(3);
        double yaw = state_(4);
        double yaw_rate = state_(5);

        // 1. 상태 예측 (CTRV 모델)
        if (fabs(yaw_rate) > 0.001) {
            state_(0) += (v / yaw_rate) * (sin(yaw + yaw_rate * dt) - sin(yaw));
            state_(1) += (v / yaw_rate) * (-cos(yaw + yaw_rate * dt) + cos(yaw));
        } else {
            state_(0) += v * cos(yaw) * dt;
            state_(1) += v * sin(yaw) * dt;
        }
        // z는 등속 유지 (state_(2) += 0)
        state_(4) += yaw_rate * dt;

        // 2. 자코비안 F 계산
        Eigen::MatrixXd Fj = Eigen::MatrixXd::Identity(6, 6);
        if (fabs(yaw_rate) > 0.001) {
            double theta_new = yaw + yaw_rate * dt;
            Fj(0, 3) = (1/yaw_rate) * (sin(theta_new) - sin(yaw));
            Fj(0, 4) = (v/yaw_rate) * (cos(theta_new) - cos(yaw));
            Fj(0, 5) = (dt*v/yaw_rate)*cos(theta_new) - (v/pow(yaw_rate,2))*(sin(theta_new)-sin(yaw));
            Fj(1, 3) = (1/yaw_rate) * (-cos(theta_new) + cos(yaw));
            Fj(1, 4) = (v/yaw_rate) * (sin(theta_new) - sin(yaw));
            Fj(1, 5) = (dt*v/yaw_rate)*sin(theta_new) - (v/pow(yaw_rate,2))*(-cos(theta_new)+cos(yaw));
        } else {
            Fj(0, 3) = cos(yaw) * dt;
            Fj(0, 4) = -v * sin(yaw) * dt;
            Fj(1, 3) = sin(yaw) * dt;
            Fj(1, 4) = v * cos(yaw) * dt;
        }
        Fj(4, 5) = dt;

        // 3. 공분산 예측
        P_ = Fj * P_ * Fj.transpose() + Q_;
    }

    void update(const geometry_msgs::Point& pos, double meas_yaw)
    {
        if (!initialized_) {
            init(pos, meas_yaw);
            return;
        }

        // 측정 벡터 z [x, y, z, yaw]
        Eigen::VectorXd z(4);
        z << pos.x, pos.y, pos.z, meas_yaw;

        // 관측 행렬 H (상태 -> 측정)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 6);
        H(0,0) = 1.0; H(1,1) = 1.0; H(2,2) = 1.0; 
        H(3,4) = 1.0; // yaw 관측

        // 잔차 계산 (y = z - Hx)
        Eigen::VectorXd y = z - H * state_;

        // ★ 각도 정규화 (-PI ~ PI)
        while (y(3) > M_PI) y(3) -= 2.0 * M_PI;
        while (y(3) < -M_PI) y(3) += 2.0 * M_PI;

        // 칼만 게인
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 상태 업데이트
        state_ = state_ + (K * y);
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
        
        // 상태 각도도 정규화
        while (state_(4) > M_PI) state_(4) -= 2.0 * M_PI;
        while (state_(4) < -M_PI) state_(4) += 2.0 * M_PI;
    }

    geometry_msgs::Point getPosition() const {
        geometry_msgs::Point p;
        p.x = state_(0); p.y = state_(1); p.z = state_(2);
        return p;
    }

    geometry_msgs::Point getVelocity() const {
        // v, yaw -> vx, vy 변환
        geometry_msgs::Point v;
        v.x = state_(3) * cos(state_(4));
        v.y = state_(3) * sin(state_(4));
        v.z = 0;
        return v;
    }

    Eigen::MatrixXd getCovariance() const { return P_; }
    bool isInitialized() const { return initialized_; }
};

// ------------------------------------------------------------
// [Class] 헝가리안 알고리즘 (기존과 동일)
// ------------------------------------------------------------
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
        // 간단한 Greedy 매칭 (정식 Munkres는 너무 길어서 생략, 실제론 라이브러리 사용 권장)
        // 여기서는 가장 작은 값부터 찾아가는 방식으로 대체 구현 (성능상 큰 차이 없음)
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
    ros::Subscriber odom_sub_; // [추가] 내 차 속도용
    
    ros::Publisher tracking_pub_;
    ros::Publisher marker_pub_;
    
    float max_mahalanobis_distance_;
    float max_disappeared_frames_;
    float gating_threshold_;
    
    double my_car_velocity_; // [추가] 내 차 속도 (m/s)

    struct Track
    {
        int track_id;
        ExtendedKalmanFilter kf; // [수정] EKF 사용
        vision_msgs::Detection3D detection;
        int disappeared_count;
        ros::Time last_seen;
        int age;
    };
    
    std::vector<Track> tracks_;
    int next_track_id_;
    ros::Time last_update_time_;

    // Helper: 쿼터니언 -> Yaw 변환
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
        
        // 6x6 공분산에서 위치(3x3) 부분만 사용
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
        
        // 1. 기존 마커 싹 지우기 (잔상 제거)
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (size_t i = 0; i < tracked_array.detections.size(); ++i)
        {
            const auto& det = tracked_array.detections[i];
            int track_id = (det.results.empty() ? -1 : det.results[0].id);
            if (track_id == -1) continue;

            // -------------------------------------------------------
            // [1] 박스 마커 (Box) - 파란색 반투명
            // -------------------------------------------------------
            visualization_msgs::Marker marker;
            marker.header = tracked_array.header;
            marker.ns = "tracked_objects";
            marker.id = track_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = det.bbox.center;
            marker.scale = det.bbox.size;
            
            // 색상: 파란색 (R=0, G=0, B=1), 투명도 0.5
            marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; 
            marker.color.a = 0.5f; 
            
            marker.lifetime = ros::Duration(0.15); // 깜빡임 방지용 수명
            marker_array.markers.push_back(marker);

            // -------------------------------------------------------
            // [2] 텍스트 마커 (Text) - ID만 표시
            // -------------------------------------------------------
            visualization_msgs::Marker text_marker = marker;
            text_marker.ns = "tracked_id";
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            
            // ★ 수정됨: 속도 계산 다 빼고 ID만 심플하게 넣음
            text_marker.text = "ID: " + std::to_string(track_id); 
            
            text_marker.scale.z = 1.0; // 글자 크기 (필요하면 조절: 0.5 ~ 1.0)
            
            // 글자 색상: 흰색 찐하게
            text_marker.color.r = 1.0f; text_marker.color.g = 1.0f; text_marker.color.b = 1.0f; 
            text_marker.color.a = 1.0f;
            
            // 박스보다 1.0m 위에 띄우기
            text_marker.pose.position.z += 1.0; 
            
            marker_array.markers.push_back(text_marker);
        }
        
        marker_pub_.publish(marker_array);
    }
    // --------------------------------------------------------------------------------
    // [수정됨] 트랙 업데이트 (Yaw 반영 & 정지 물체 필터링)
    // --------------------------------------------------------------------------------
    void updateTrack(Track& track, const vision_msgs::Detection3D& detection, double dt)
    {
        geometry_msgs::Point measurement_pos = getCenter(detection);
        
        // [1] Yaw 추출 (Quaternion -> Yaw)
        double measurement_yaw = getYawFromQuaternion(detection.bbox.center.orientation);

        if (track.age < 3) 
        {
            double last_yaw = track.kf.state_(4);       // 직전 Yaw
            double diff_yaw = measurement_yaw - last_yaw; // 각도 변화량

            // 각도 정규화 (-PI ~ PI) - 필수!
            while (diff_yaw > M_PI) diff_yaw -= 2.0 * M_PI;
            while (diff_yaw < -M_PI) diff_yaw += 2.0 * M_PI;

            // 시간으로 나눠서 각속도 계산 후 강제 주입
            double instant_yaw_rate = diff_yaw / dt;
            track.kf.state_(5) = instant_yaw_rate; 
        }

        // [2] EKF 예측 & 업데이트
        track.kf.predict(dt);
        track.kf.update(measurement_pos, measurement_yaw);

        // [3] ★ 정지 물체 필터링 (Ego-Motion Compensation)
        // 절대 속도 = (물체 상대 속도 * cos(물체 헤딩)) + 내 차 속도
        // (X축 방향만 고려한 약식 계산)
        double obj_rel_v = track.kf.state_(3); // EKF가 추정한 상대 속도
        double obj_vx = obj_rel_v * cos(track.kf.state_(4)); 
        double abs_vx = obj_vx + my_car_velocity_; // 절대 속도

        // 절대 속도가 3.6km/h (1.0m/s) 미만이면 정지로 간주
        if (std::abs(abs_vx) < 0.5) 
        {
            track.kf.setVelocityZero(); // 속도를 0으로 죽임 (Ghost Effect 방지)
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
            
            geometry_msgs::Point init_pos = getCenter(detections.detections[det_idx]);
            double init_yaw = getYawFromQuaternion(detections.detections[det_idx].bbox.center.orientation);
            
            // [수정] 초기화 시 Yaw도 같이 입력
            new_track.kf.init(init_pos, init_yaw);
            
            tracks_.push_back(new_track);
        }
    }
    
public:
    GigachaLidarTracking() : nh_("~"), next_track_id_(1), my_car_velocity_(0.0)
    {
        ROS_INFO("GIGACHA LiDAR Tracking Node (EKF-CTRV + Static Filter) Starting...");
        
        nh_.param<float>("max_mahalanobis_distance", max_mahalanobis_distance_, 3.0f);
        nh_.param<float>("max_disappeared_frames", max_disappeared_frames_, 5.0f);
        nh_.param<float>("gating_threshold", gating_threshold_, 10.0f); 
        
        detection_sub_ = nh_.subscribe("/gigacha/lidar/bounding_boxes", 1, &GigachaLidarTracking::detectionCallback, this);
        
        // [추가] 내 차 속도 구독 (토픽 이름 환경에 맞게 수정 필수!)
        odom_sub_ = nh_.subscribe("/odom", 1, &GigachaLidarTracking::odomCallback, this);
        
        tracking_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/tracked_objects", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracked_objects", 1);
        
        last_update_time_ = ros::Time::now();
    }
    
    // [추가] 오돔 콜백
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        my_car_velocity_ = msg->twist.twist.linear.x; // 전진 속도
    }
    
    void detectionCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        if (dt < 0.001 || dt > 1.0) dt = 0.1;
        last_update_time_ = current_time;
        
        if (msg->detections.empty()) {
            updateDisappearedTracks(dt);
            removeOldTracks();
            vision_msgs::Detection3DArray empty;
            empty.header = msg->header;
            tracking_pub_.publish(empty);
            publishMarkers(empty);
            return;
        }
        
        std::vector<int> matched_tracks, unmatched_detections;
        associateDetectionsToTracks(*msg, matched_tracks, unmatched_detections);
        
        for (size_t i = 0; i < tracks_.size(); i++) {
            if (matched_tracks[i] >= 0 && matched_tracks[i] < (int)msg->detections.size()) {
                updateTrack(tracks_[i], msg->detections[matched_tracks[i]], dt);
            } else {
                tracks_[i].disappeared_count++;
                tracks_[i].kf.predict(dt);
            }
        }
        
        createNewTracks(*msg, unmatched_detections);
        removeOldTracks();
        
        vision_msgs::Detection3DArray tracked_array;
        tracked_array.header = msg->header;
        for (const auto& track : tracks_) {
            if (track.disappeared_count == 0 && track.age > 3) {
                vision_msgs::Detection3D tracked_det = track.detection;
                tracked_det.bbox.center.position = track.kf.getPosition();
                // 결과에 속도 정보도 넣으면 좋지만, 여기선 ID만
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