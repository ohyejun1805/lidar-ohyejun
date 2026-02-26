#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h> 
#include <morai_msgs/EgoVehicleStatus.h> 

#include <limits>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

// ==============================================================================
// [각주 1: AB3DMOT의 핵심, 선형 칼만 필터]
// 기존의 복잡한 비선형 확장 칼만 필터(EKF) 대신 선형(Standard) 칼만 필터를 사용합니다.
// AB3DMOT 논문에서는 연산 속도와 안정성을 위해 3D 박스의 '크기'까지 필터링합니다.
// - 상태 변수(10차원): [x, y, z, yaw, l, w, h, v_x, v_y, v_z]
// - 관측 변수(7차원): PointPillars에서 넘어오는 [x, y, z, yaw, l, w, h]
// ==============================================================================
class StandardKalmanFilter
{
public: 
    Eigen::VectorXd state_; 
private:
    Eigen::MatrixXd P_; // 오차 공분산 (현재 추정치에 대한 불확실성)
    Eigen::MatrixXd F_; // 상태 전이 행렬 (시간에 따른 상태 변화 모델)
    Eigen::MatrixXd H_; // 관측 행렬 (상태 변수에서 관측치로의 변환)
    Eigen::MatrixXd Q_; // 시스템 노이즈 (예측 모델이 얼마나 부정확한지)
    Eigen::MatrixXd R_; // 센서 관측 노이즈 (PointPillars 박스가 얼마나 흔들리는지)
    bool initialized_;

public:
    StandardKalmanFilter() : initialized_(false)
    {
        state_ = Eigen::VectorXd::Zero(10);
        P_ = Eigen::MatrixXd::Identity(10, 10) * 10.0;
        
        // 등속 직선 운동 모델 설정 (F 행렬)
        F_ = Eigen::MatrixXd::Identity(10, 10);
        
        // 관측치는 앞의 7개(위치, yaw, 크기)만 들어옵니다.
        H_ = Eigen::MatrixXd::Zero(7, 10);
        for(int i = 0; i < 7; i++) H_(i, i) = 1.0;

        // 노이즈 파라미터 (실제 주행 시 이 값들을 튜닝하여 필터 반응성을 조절합니다)
        Q_ = Eigen::MatrixXd::Identity(10, 10) * 0.1; 
        R_ = Eigen::MatrixXd::Identity(7, 7) * 0.1; 
    }

    // 객체가 처음 발견되었을 때 칼만 필터를 초기화합니다.
    void init(const vision_msgs::Detection3D& det, double yaw)
    {
        state_ << det.bbox.center.position.x, 
                  det.bbox.center.position.y, 
                  det.bbox.center.position.z, 
                  yaw, 
                  det.bbox.size.x, // l (길이)
                  det.bbox.size.y, // w (너비)
                  det.bbox.size.z, // h (높이)
                  0.0, 0.0, 0.0;   // 초기 속도는 0
        initialized_ = true;
    }

    void setVelocityZero() 
    {
        state_(7) = 0.0; 
        state_(8) = 0.0; 
        state_(9) = 0.0;
    }

    // 예측 단계
    // 이전 프레임의 위치와 속도를 바탕으로 현재 프레임에서 객체가 어디 있을지 예측합니다.
    void predict(double dt)
    {
        if (!initialized_) return;

        // 위치 = 기존 위치 + 속도 * 시간(dt)
        F_(0, 7) = dt; // x = x + v_x * dt
        F_(1, 8) = dt; // y = y + v_y * dt
        F_(2, 9) = dt; // z = z + v_z * dt

        state_ = F_ * state_;
        P_ = F_ * P_ * F_.transpose() + Q_;
        
        // Yaw 값이 -PI ~ PI 사이를 유지하도록 정규화
        while (state_(3) > M_PI) state_(3) -= 2.0 * M_PI;
        while (state_(3) < -M_PI) state_(3) += 2.0 * M_PI;
    }

    // 업데이트 단계
    // PointPillars에서 실제 측정한 값을 바탕으로 예측값을 교정합니다.
    void update(const vision_msgs::Detection3D& det, double meas_yaw)
    {
        if (!initialized_) {
            init(det, meas_yaw);
            return;
        }

        Eigen::VectorXd z(7);
        z << det.bbox.center.position.x, 
             det.bbox.center.position.y, 
             det.bbox.center.position.z, 
             meas_yaw,
             det.bbox.size.x,
             det.bbox.size.y,
             det.bbox.size.z;

        // 실제 측정값(z)과 예측값(H * state)의 차이(Innovation)
        Eigen::VectorXd y = z - H_ * state_;

        //각도 정규화
        while (y(3) > M_PI) y(3) -= 2.0 * M_PI;
        while (y(3) < -M_PI) y(3) += 2.0 * M_PI;

        // 칼만 게인 계산 및 상태 업데이트
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        state_ = state_ + (K * y);
        P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H_) * P_;
        
        while (state_(3) > M_PI) state_(3) -= 2.0 * M_PI;
        while (state_(3) < -M_PI) state_(3) += 2.0 * M_PI;
    }

    geometry_msgs::Point getPosition() const 
    {
        geometry_msgs::Point p;
        p.x = state_(0); p.y = state_(1); p.z = state_(2);
        return p;
    }
    
    geometry_msgs::Vector3 getDimensions() const
    {
        geometry_msgs::Vector3 dim;
        dim.x = state_(4); dim.y = state_(5); dim.z = state_(6);
        return dim;
    }

    bool isInitialized() const { return initialized_; }
};


// 헝가리안 알고리즘
class HungarianAlgorithm {
    // (내부 구현은 수정 없이 기존과 100% 동일하게 사용)
public:
    static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix) {
        if (cost_matrix.empty() || cost_matrix[0].empty()) return std::vector<int>(cost_matrix.size(), -1);
        int n = cost_matrix.size(); int m = cost_matrix[0].size(); int size = std::max(n, m);
        std::vector<std::vector<double>> matrix(size, std::vector<double>(size, 0.0));
        for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) matrix[i][j] = cost_matrix[i][j];
        std::vector<int> assignment = munkres(matrix);
        std::vector<int> result(n, -1);
        for (int i = 0; i < n && i < size; i++) if (assignment[i] < m) result[i] = assignment[i];
        return result;
    }
private:
    static std::vector<int> munkres(std::vector<std::vector<double>>& matrix) {
        int n = matrix.size();
        for (int i = 0; i < n; i++) { double min_val = *std::min_element(matrix[i].begin(), matrix[i].end()); for (int j = 0; j < n; j++) matrix[i][j] -= min_val; }
        for (int j = 0; j < n; j++) { double min_val = matrix[0][j]; for (int i = 1; i < n; i++) min_val = std::min(min_val, matrix[i][j]); for (int i = 0; i < n; i++) matrix[i][j] -= min_val; }
        std::vector<int> assignment(n, -1); std::vector<bool> row_covered(n, false); std::vector<bool> col_covered(n, false);
        for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) { if (matrix[i][j] == 0.0 && !row_covered[i] && !col_covered[j]) { assignment[i] = j; row_covered[i] = true; col_covered[j] = true; break; } } }
        return assignment;
    }
};

// ==============================================================================
// 메인 트래킹 클래스
// ==============================================================================
class GigachaLidarTracking
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber detection_sub_;
    ros::Subscriber ego_sub_; 
    
    ros::Publisher tracking_pub_;
    ros::Publisher marker_pub_;
    
    float min_iou_threshold_; // IoU 임계값
    float max_disappeared_frames_; // 객체가 사라진 후 몇 프레임까지 기억할 것인지
    
    double my_car_velocity_; 

    // 개별 객체의 추적 정보를 담는 구조체
    struct Track
    {
        int track_id;
        StandardKalmanFilter kf;
        vision_msgs::Detection3D detection;
        int disappeared_count;
        ros::Time last_seen;
        int age; // 객체가 몇 프레임 연속으로 보였는지
        
        bool has_moved;       
        bool is_large_size;   
    };
    
    std::vector<Track> tracks_;
    int next_track_id_;
    ros::Time last_update_time_;

    // 쿼터니언에서 Yaw 각도를 추출하는 함수
    double getYawFromQuaternion(const geometry_msgs::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // ==============================================================================
    // [각주 4: 3D IoU (Intersection over Union) 계산]
    // AB3DMOT의 핵심인 Data Association 기준입니다. 두 3D 박스가 겹치는 비율을 계산합니다.
    // 연산 속도를 위해 차량의 회전(Yaw)을 배제하고 AABB(축 정렬 바운딩 박스) 방식으로 근사 계산합니다.
    // 자율주행 특성상 프레임 간격이 짧아 객체의 회전 변화량이 적기 때문에 충분히 잘 작동합니다.
    // ==============================================================================
    
    double calculate3DIoU(const geometry_msgs::Point& p1, const geometry_msgs::Vector3& d1,
                          const geometry_msgs::Point& p2, const geometry_msgs::Vector3& d2)
    {
        // X, Y, Z 각 축에 대해 겹치는 길이 계산
        double x_overlap = std::max(0.0, std::min(p1.x + d1.x/2.0, p2.x + d2.x/2.0) - std::max(p1.x - d1.x/2.0, p2.x - d2.x/2.0));
        double y_overlap = std::max(0.0, std::min(p1.y + d1.y/2.0, p2.y + d2.y/2.0) - std::max(p1.y - d1.y/2.0, p2.y - d2.y/2.0));
        double z_overlap = std::max(0.0, std::min(p1.z + d1.z/2.0, p2.z + d2.z/2.0) - std::max(p1.z - d1.z/2.0, p2.z - d2.z/2.0));

        // 교집합 부피
        double intersection_vol = x_overlap * y_overlap * z_overlap;

        // 합집합 부피 = 부피1 + 부피2 - 교집합 부피
        double vol1 = d1.x * d1.y * d1.z;
        double vol2 = d2.x * d2.y * d2.z;
        double union_vol = vol1 + vol2 - intersection_vol;

        if (union_vol <= 0.0001) return 0.0;
        return intersection_vol / union_vol; // 0.0 ~ 1.0 사이의 값 반환
    }
    
    // ==============================================================================
    // [각주 5: Cost Matrix (비용 행렬) 생성]
    // 기존 마할라노비스 거리 대신 3D IoU를 사용하여 비용을 계산합니다.
    // ==============================================================================
    std::vector<std::vector<double>> buildCostMatrix(const vision_msgs::Detection3DArray& detections)
    {
        std::vector<std::vector<double>> cost_matrix;
        if (tracks_.empty() || detections.detections.empty()) return cost_matrix;
        
        cost_matrix.resize(tracks_.size());
        for (size_t i = 0; i < tracks_.size(); i++)
        {
            cost_matrix[i].resize(detections.detections.size());
            
            // 칼만 필터가 '예측'한 객체의 위치와 크기
            geometry_msgs::Point pred_pos = tracks_[i].kf.getPosition();
            geometry_msgs::Vector3 pred_dim = tracks_[i].kf.getDimensions();
            
            for (size_t j = 0; j < detections.detections.size(); j++)
            {
                // PointPillars가 '측정'한 객체의 위치와 크기
                geometry_msgs::Point meas_pos = detections.detections[j].bbox.center.position;
                geometry_msgs::Vector3 meas_dim = detections.detections[j].bbox.size;

                double iou = calculate3DIoU(pred_pos, pred_dim, meas_pos, meas_dim);
                
                // 헝가리안 알고리즘은 '비용(Cost)이 낮을수록' 매칭을 우선시합니다.
                // IoU는 높을수록(1에 가까울수록) 좋은 것이므로, Cost = 1.0 - IoU 로 설정합니다.
                if (iou < min_iou_threshold_) cost_matrix[i][j] = 1e6; // 아예 안 겹치면 매칭 금지
                else cost_matrix[i][j] = 1.0 - iou;
            }
        }
        return cost_matrix;
    }
    
    // 이전 프레임의 트랙들과 현재 프레임의 Detection 결과를 매칭
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
                // 매칭은 되었으나 IoU가 너무 낮다면 매칭 취소
                if (cost_matrix[i][matched_tracks[i]] > (1.0 - min_iou_threshold_)) {
                    matched_tracks[i] = -1;
                } else {
                    det_matched[matched_tracks[i]] = true;
                }
            }
        }
        
        for (size_t j = 0; j < detections.detections.size(); j++)
            if (!det_matched[j]) unmatched_detections.push_back(j);
    }

    // RViz 시각화용 마커 퍼블리시
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

            // Bounding Box 마커
            visualization_msgs::Marker marker;
            marker.header = tracked_array.header;
            marker.ns = "tracked_objects";
            marker.id = track_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = det.bbox.center;
            
            marker.pose.orientation = det.bbox.center.orientation;
            marker.scale = det.bbox.size;
            
            marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; 
            marker.color.a = 0.5f; 
            marker.lifetime = ros::Duration(0.15); 
            marker_array.markers.push_back(marker);

            // ID 텍스트 마커
            visualization_msgs::Marker text_marker = marker;
            text_marker.ns = "tracked_id";
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.text = "ID: " + std::to_string(track_id); 
            text_marker.scale.z = 1.0; 
            text_marker.color.r = 1.0f; text_marker.color.g = 1.0f; text_marker.color.b = 1.0f; 
            text_marker.color.a = 1.0f;
            text_marker.pose.position.z += (det.bbox.size.z / 2.0) + 0.5; // 박스 위쪽에 표시
            marker_array.markers.push_back(text_marker);
        }
        
        marker_pub_.publish(marker_array);
    }

    // 매칭된 트랙의 칼만 필터 업데이트
    void updateTrack(Track& track, const vision_msgs::Detection3D& detection, double dt)
    {
        double measurement_yaw = getYawFromQuaternion(detection.bbox.center.orientation);

        double track_vx = track.kf.state_(7);
        double track_vy = track.kf.state_(8);
        double track_v = std::sqrt(track_vx * track_vx + track_vy * track_vy);
        double track_yaw = track.kf.state_(3);

        // 빠른 속도로 움직이는 객체의 경우, 방향(Yaw)이 180도 뒤집히는 현상 방지
        if (track_v > 2.0) 
        {
            double vel_angle = track_yaw; 
            double diff = measurement_yaw - vel_angle;
            
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;

            double abs_diff = std::abs(diff);

            if (abs_diff > (M_PI * 0.75)) measurement_yaw += M_PI;
            else if (abs_diff > (M_PI * 0.25) && abs_diff < (M_PI * 0.75)) measurement_yaw = vel_angle; 
            
            while (measurement_yaw > M_PI) measurement_yaw -= 2.0 * M_PI;
            while (measurement_yaw < -M_PI) measurement_yaw += 2.0 * M_PI;
        }

        // 칼만 필터 예측 및 업데이트 수행
        track.kf.predict(dt);
        track.kf.update(detection, measurement_yaw);

        // 절대 속도 계산 (내 차량 속도 보상)
        double obj_rel_vx = track.kf.state_(7); 
        double obj_rel_vy = track.kf.state_(8); 
        double abs_vx = obj_rel_vx + my_car_velocity_; 
        double abs_vy = obj_rel_vy;
        double abs_v_mag = std::sqrt(abs_vx * abs_vx + abs_vy * abs_vy);

        // 거의 멈춰있는 객체는 속도를 0으로 강제 보정 (노이즈로 인해 속도가 튀는 것 방지)
        if (abs_v_mag < 1.0) track.kf.setVelocityZero(); 
        
        if (track.age > 3 && abs_v_mag > 1.5) track.has_moved = true; 

        float max_s = std::max(detection.bbox.size.x, detection.bbox.size.y);
        track.is_large_size = (max_s > 5.8f);

        track.detection = detection;
        track.disappeared_count = 0; // 매칭 성공 시 카운트 초기화
        track.last_seen = ros::Time::now();
        track.age++; // 객체가 연속으로 보인 프레임 수 증가
    }
    
    // 매칭되지 않은(사라진) 트랙들을 추측으로 이동시킴
    void updateDisappearedTracks(double dt)
    {
        for (auto& track : tracks_)
            if (track.disappeared_count > 0) track.kf.predict(dt);
    }
    
    // 너무 오래 안 보이는 객체 삭제 (메모리 관리)
    void removeOldTracks()
    {
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const Track& track) {
                return track.disappeared_count > max_disappeared_frames_;
            }), tracks_.end());
    }
    
    // 새롭게 발견된 객체에 ID 부여 및 트랙 생성
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

            double init_yaw = getYawFromQuaternion(detections.detections[det_idx].bbox.center.orientation);
            new_track.kf.init(detections.detections[det_idx], init_yaw);
            
            tracks_.push_back(new_track);
        }
    }
    
public:
    GigachaLidarTracking() : nh_("~"), next_track_id_(1), my_car_velocity_(0.0)
    {
        ROS_INFO("GIGACHA LiDAR Tracking Node (PointPillars + AB3DMOT) Starting...");
        
        // IoU가 0.1 이하이면 다른 객체로 간주
        nh_.param<float>("min_iou_threshold", min_iou_threshold_, 0.1f);
        // 5프레임 연속으로 안 보이면 추적 포기
        nh_.param<float>("max_disappeared_frames", max_disappeared_frames_, 5.0f);
        
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
    
    // PointPillars 결과가 들어올 때마다 실행되는 메인 콜백
    void detectionCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        if (dt < 0.001 || dt > 1.0) dt = 0.1;
        last_update_time_ = current_time;
        
        vision_msgs::Detection3DArray processed_msg = *msg; 
        std::vector<int> valid_indices; 

        // Size Filtering
        for (size_t i = 0; i < processed_msg.detections.size(); ++i)
        {
            float len = processed_msg.detections[i].bbox.size.x;
            float wid = processed_msg.detections[i].bbox.size.y;
            float hgt = processed_msg.detections[i].bbox.size.z;

            float min_side = std::min(len, wid); 
            float max_side = std::max(len, wid); 
            float safe_min_side = std::max(min_side, 0.01f); 
            float aspect_ratio = max_side / safe_min_side;

            // 너무 작거나 크거나, 비정상적으로 납작한 노이즈 컷
            if (max_side < 0.2f) continue;   
            if (max_side > 20.0f) continue;  
            if (hgt < 0.5f) continue;    

            bool is_person = (max_side < 1.0f) && (min_side < 0.8f);
            bool is_car = (min_side >= 0.2f && min_side <= 2.5f) && (max_side >= 1.2f && max_side <= 5.8f);
            bool is_large = (max_side > 5.8f && max_side <= 19.0f);

            if (is_person) {} 
            else if (is_car) { if (aspect_ratio > 15.0f) continue; }
            else if (is_large) { if (hgt < 2.2f || hgt > 4.5f) continue; }
            else continue;

            valid_indices.push_back(i);
        }

        // 필터링을 통과한 유효한 박스들만 새 배열에 담습니다.
        vision_msgs::Detection3DArray filtered_msg;
        filtered_msg.header = msg->header;
        for(int idx : valid_indices) filtered_msg.detections.push_back(processed_msg.detections[idx]);

        // 만약 이번 프레임에 잡힌 객체가 하나도 없다면?
        if (filtered_msg.detections.empty()) {
            updateDisappearedTracks(dt); // 기존 객체들은 예측 위치로만 이동시키고
            removeOldTracks();           // 너무 오래 안 보인 객체 지우고
            
            vision_msgs::Detection3DArray empty;
            empty.header = msg->header;
            tracking_pub_.publish(empty);
            publishMarkers(empty);
            return;
        }
        
        // 1. Data Association (3D IoU + 헝가리안 매칭)
        std::vector<int> matched_tracks, unmatched_detections;
        associateDetectionsToTracks(filtered_msg, matched_tracks, unmatched_detections);
        
        // 2. State Update (칼만 필터 갱신)
        for (size_t i = 0; i < tracks_.size(); i++) {
            if (matched_tracks[i] >= 0 && matched_tracks[i] < (int)filtered_msg.detections.size()) {
                updateTrack(tracks_[i], filtered_msg.detections[matched_tracks[i]], dt);
            } else {
                tracks_[i].disappeared_count++; // 매칭 실패 시 카운트 증가
                tracks_[i].kf.predict(dt);
            }
        }
        
        // 3. Track Management (트랙 생성 및 소멸)
        createNewTracks(filtered_msg, unmatched_detections);
        removeOldTracks();
        
        // 4. 결과 퍼블리시
        vision_msgs::Detection3DArray tracked_array;
        tracked_array.header = msg->header;
        
        for (const auto& track : tracks_) 
        {
            // 방금 매칭 성공했고 & 최소 3프레임 이상 연속으로 잡힌 안정적인 객체만 내보냄 (Birth Process)
            if (track.disappeared_count == 0 && track.age > 3) 
            {
                bool should_publish = true; 

                // 대형 객체(버스, 트럭 등)는 한 번이라도 움직인 적이 있을 때만 내보냄 (정적 노이즈 방지)
                if (track.is_large_size && !track.has_moved) should_publish = false; 

                if (should_publish) 
                {
                    vision_msgs::Detection3D tracked_det = track.detection;
                    // 위치를 단순히 센서 측정값이 아닌, 칼만 필터가 부드럽게 보정한 위치로 덮어씌움
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