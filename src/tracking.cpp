#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
// lidar bbox 데이터를 받아오기 위한 메시지 타입

#include <visualization_msgs/MarkerArray.h>
//Rviz에 ID나 궤적을 표시하기 위한 메시지 타입

#include <geometry_msgs/Point.h>
//Ros에서 3D 포인트를 표현하기 위한 메시지 타입

#include <Eigen/Dense>
//c++ 선형대수 라이브러리, 행렬 벡터 연산 등에 사용
//칼만 필터 대부분이 행렬 연산이기 때문에 Eigen 라이브러리를 사용
#include <vector>
#include <cmath>
#include <limits>
#include <string>


//Kalman Filter 클래스 정의
class KalmanFilter3D {
public:
    // [최적화] 동적 할당(MatrixXd) 대신 고정 크기(Matrix<double, ...>) 사용으로 속도 향상
    Eigen::Matrix<double, 6, 6> I_;
    //단위 행렬
    Eigen::Matrix<double, 6, 1> x_; 
    //상태 벡터 [x, y, z, vx, vy, vz] , 우리가 알고 싶은 위치와 속도 정보
    //벡터 크기 6, 위치 3개, 속도 3개
    Eigen::Matrix<double, 6, 6> P_; 
    //오차 공분산 행렬, 내 추정값에 대한 불확실성
    //6x6 행렬, 이 값이 크면 내 추정이 불확실하다는 뜻
    Eigen::Matrix<double, 6, 6> F_; 
    //상태 전이 행렬, 물리 법칙.
    //과거 상태로 현재 상태를 예측할 때 사용
    Eigen::Matrix<double, 6, 6> Q_; 
    //프로세스 노이즈, 현실 변수임. 예측 불가능한 것.
    //앞차 급제동이나, 미끄러운 도로 등 수식으로 예측 불가능한 외란.
    Eigen::Matrix<double, 3, 6> H_; 
    //측정 행렬, 상태 변수와 센서 데이터 간의 연결고리.
    //lidar 센서는 3개 값만 주는데, 이걸 우리의 6개 상태 변수로 맞춰주는 행렬
    //그래서 3x6 행렬임.
    Eigen::Matrix<double, 3, 3> R_;
    //측정 노이즈 행렬, 센서 데이터의 불확실성
    //비싼 lidar 쓸수록 이 값이 작아짐. 

    //생성자
    KalmanFilter3D() {
        
        I_.setIdentity();

        x_.setZero();
        
        P_.setIdentity(); 
        //Eigen 라이브러리 제공 함수.
        //P_를 단위 행렬로 만들어주는 함수.
        //P가 주대각성분만 1이고 나머지는 0인 행렬이 됨.
        P_ *= 1000.0;
        //초기 불확실성을 굉장히 크게 잡음.
        //주대각 성분 다 1000 곱해줌.

        F_.setIdentity();
        //처음에는 dt가 0이기 때문에 단위행렬로 초기화
        //일단 세상이 정지해 있다고 가정
        //나중에 predict 함수에서 dt에 따라 F_를 업데이트 해줌

        H_.setZero();//다 0 넣고
        H_(0, 0) = 1.0;//(0,0) (1,1) (2,2) 에 1 넣어줌
        H_(1, 1) = 1.0;
        H_(2, 2) = 1.0;

        Q_.setIdentity();//프로세스 노이즈 행렬 단위행렬로 초기화
        Q_ *= 0.1; //0.1 곱해줌
        
        R_.setIdentity();//측정 노이즈 행렬 단위행렬로 초기화
        R_ *= 0.1; //0.1 곱해줌
    }

    //예측
    void predict(double dt) {//dt는 lidar 데이터 들어오는 주기
        F_(0, 3) = dt;//노션 참고
        F_(1, 4) = dt;
        F_(2, 5) = dt;

        // [최적화] x_ = F_ * x_; 대신 직접 연산 사용
        // 행렬 곱셈(36번의 곱셈) 대신 3번의 덧셈/곱셈으로 줄임
        //상태 예측, 위치는 속도에 dt 곱한 값만큼 증가
        //등속도 운동 가정. 
        x_(0) += dt * x_(3);
        x_(1) += dt * x_(4);
        x_(2) += dt * x_(5);
        // 속도(3,4,5)는 등속 모델이라 변화 없음

        P_ = F_ * P_ * F_.transpose() + Q_;
        //차가 이동했으니까, 오차 공분산 행렬도 업데이트
        //속도를 잘 모르면, 시간이 지날수록 위치 오차도 더 커진다.를 반영해 오차 풍선 커짐.
        //그냥 이상적 상황 계산이고, 여기에 예측 불가능한 노이즈 Q_ 더해줌.
    }

    //상상한 값과 실제 측정값 비교해서 보정하고, 
    //진짜 내 위치와 속도에 더 가까운 값으로 수정
    void update(const Eigen::Vector3d& z) {
        //inovation or 잔차
        Eigen::Vector3d y = z - H_ * x_; 
        //lidar로 측정한 위치 z와 우리 예측값 H_ * x_의 차이 y.
        Eigen::Matrix<double, 3, 3> S = H_ * P_ * H_.transpose() + R_; 
        //전차 공분산 S,
        //예측 불확실성 P_를 lidar 측정 공간 3x3로 변환하고, 센서 노이즈 R_ 더해줌.
        
        // [최적화] 역행렬 구하기
        Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();
        /*칼만 게인 K, 제일 중요한 부분!!!!!
        센서 데이터를 얼마나 신뢰할지 결정하는 값.
        내 에측이 더 정확하면 K가 작아지고, 센서가 더 정확하면 K가 커짐.
        */

        x_ = x_ + K * y;
        //내 최종 상태 업데이트
        P_ = (I_ - K * H_) * P_;
        //오차 공분산 행렬 업데이트
        //센서를 더 신뢰하면(K가 크면) 오차 공분산 P_가 더 작게 업데이트.
        //예측했을 때는 불확실성이 컸는데(P_ 풍선이 컸는데) lidar 센서가 정확해서
        //내 위치를 잘 보정해줬으니까, 이제 내 위치에 대한 불확실성은 줄어듦.
    }
};

//Track되는 객체의 life cycle 관리 클래스
//도로 위에 객체가 10개면, 10개의 Track 객체가 생성되어 각각의 Kalman Filter를 가짐
class Track {
public:
    int id_;
    int age_;          
    int total_visible_count_; 
    int consecutive_invisible_count_;
    
    KalmanFilter3D kf_;

    Track(int id, const Eigen::Vector3d& init_pos) : id_(id) {
        age_ = 0;
        total_visible_count_ = 1;
        consecutive_invisible_count_ = 0;

        kf_.x_.head(3) = init_pos;
    }

    Eigen::Vector3d getPosition() {
        return kf_.x_.head(3);
    }

    Eigen::Vector3d getVelocity() {
        return kf_.x_.tail(3);
    }
};

class GigachaLidarTracking {
private:
    ros::NodeHandle nh_;
    ros::Subscriber detection_sub_;
    ros::Publisher marker_pub_;

    std::vector<Track> tracks_;
    int next_track_id_;
    
    double dist_threshold_; 
    double dist_threshold_sq_; // [최적화] 거리 제곱 비교를 위한 변수 추가
    int max_invisible_count_; 
    int min_hits_; 

    ros::Time last_time_;

public:
    GigachaLidarTracking() : nh_("~"), next_track_id_(0) {
        ROS_INFO("GIGACHA LiDAR Tracking Node Starting...");

        nh_.param<double>("dist_threshold", dist_threshold_, 2.0);
        nh_.param<int>("max_invisible_count", max_invisible_count_, 5);
        nh_.param<int>("min_hits", min_hits_, 3);

        // [최적화] 반복문 안에서 매번 곱하지 않도록 미리 제곱해둠
        dist_threshold_sq_ = dist_threshold_ * dist_threshold_;

        detection_sub_ = nh_.subscribe("/gigacha/lidar/bounding_boxes", 1, &GigachaLidarTracking::callback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/gigacha/lidar/tracking_markers", 1);
        
        last_time_ = ros::Time::now();

        ROS_INFO("GIGACHA LiDAR Tracking Initialized");
    }

    void callback(const vision_msgs::Detection3DArray::ConstPtr& msg) {
        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        
        if (dt < 0.001) dt = 0.1; 

        for (auto& track : tracks_) {
            track.kf_.predict(dt);
        }

        std::vector<Eigen::Vector3d> measurements;
        // [최적화] vector 메모리 재할당 방지를 위해 reserve 사용
        measurements.reserve(msg->detections.size());

        for (const auto& det : msg->detections) {
            measurements.emplace_back(
                det.bbox.center.position.x,
                det.bbox.center.position.y,
                det.bbox.center.position.z
            );
        }

        std::vector<bool> measurement_matched(measurements.size(), false);
        std::vector<bool> track_matched(tracks_.size(), false);

        for (size_t i = 0; i < tracks_.size(); ++i) {
            double min_dist_sq = std::numeric_limits<double>::max(); // [최적화] 제곱 거리 사용
            int best_match_idx = -1;

            Eigen::Vector3d track_pos = tracks_[i].getPosition();

            for (size_t j = 0; j < measurements.size(); ++j) {
                if (measurement_matched[j]) continue; 

                // [최적화] Gating: 단순 차이가 threshold보다 크면 정밀 계산 스킵
                if (std::abs(track_pos.x() - measurements[j].x()) > dist_threshold_) continue;
                if (std::abs(track_pos.y() - measurements[j].y()) > dist_threshold_) continue;

                // [최적화] sqrt 연산이 없는 squaredNorm 사용
                double dist_sq = (track_pos - measurements[j]).squaredNorm();
                
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best_match_idx = j;
                }
            }

            // [최적화] 비교도 제곱된 값끼리 비교
            if (best_match_idx != -1 && min_dist_sq < dist_threshold_sq_) {
                tracks_[i].kf_.update(measurements[best_match_idx]);
                tracks_[i].consecutive_invisible_count_ = 0;
                tracks_[i].total_visible_count_++;
                
                track_matched[i] = true;
                measurement_matched[best_match_idx] = true;
            }
        }

        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (!track_matched[i]) {
                tracks_[i].consecutive_invisible_count_++;
            }
        }

        for (size_t j = 0; j < measurements.size(); ++j) {
            if (!measurement_matched[j]) {
                Track new_track(next_track_id_++, measurements[j]);
                tracks_[i].consecutive_invisible_count_ = 0; // 초기화
                tracks_.push_back(new_track);
            }
        }

        auto it = std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const Track& t) {
                return t.consecutive_invisible_count_ > max_invisible_count_;
            });
        tracks_.erase(it, tracks_.end());

        publishMarkers(msg->header);
    }

    void publishMarkers(const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (tracks_[i].total_visible_count_ < min_hits_) continue;

            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "ids";
            marker.id = tracks_[i].id_;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;

            Eigen::Vector3d pos = tracks_[i].getPosition();
            marker.pose.position.x = pos(0);
            marker.pose.position.y = pos(1);
            marker.pose.position.z = pos(2) + 1.0; 
            marker.pose.orientation.w = 1.0;

            marker.scale.z = 0.5; 
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0; 

            marker.text = "ID: " + std::to_string(tracks_[i].id_);
            marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(marker);
        }
        
        marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gigacha_lidar_tracking");
    GigachaLidarTracking node;
    ros::spin();
    return 0;
}