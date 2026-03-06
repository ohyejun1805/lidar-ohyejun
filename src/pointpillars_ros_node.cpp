#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <cmath>

// 우리가 깎아낸 핵심 엔진의 머리(헤더) 파일
#include "pointpillar.h" 

class PointPillarsROS {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Publisher pub_bboxes_;
    PointPillar* pointpillar_;
    cudaStream_t stream_;

public:
    PointPillarsROS() {
        // 1. 방금 구운 뜨끈뜨끈한 엔진 파일 경로 연결
        std::string engine_path = "/home/inji2/workspace/CUDA-PointPillars/model/pointpillar.plan";
        
        // 2. GPU 고속도로(CUDA Stream) 개통 및 엔진 시동
        cudaStreamCreate(&stream_);
        pointpillar_ = new PointPillar(engine_path, stream_);

        // 3. 통신망 개설
        // 주의: MORAI 시뮬레이터에서 쏘는 라이다 토픽 이름으로 맞춰주세요! (예: /velodyne_points)
        sub_lidar_ = nh_.subscribe("/velodyne_points", 1, &PointPillarsROS::lidarCallback, this);
        pub_bboxes_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        
        ROS_INFO("🔥🔥 GIGACHA PointPillars Deep Learning Engine Started! 🔥🔥");
    }

    ~PointPillarsROS() {
        delete pointpillar_;
        cudaStreamDestroy(stream_);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // ROS PointCloud2 메시지를 PCL 형식으로 변환
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // PointPillars 엔진이 먹기 좋게 1열 횡대(float 배열)로 줄 세우기 (x, y, z, intensity)
        std::vector<float> points_data;
        points_data.reserve(pcl_cloud->points.size() * 4);
        for (const auto& pt : pcl_cloud->points) {
            points_data.push_back(pt.x);
            points_data.push_back(pt.y);
            points_data.push_back(pt.z);
            points_data.push_back(pt.intensity);
        }

        // 🚀 대망의 딥러닝 추론 실행 (방금 구운 엔진 사용!)
        std::vector<Bndbox> nms_pred;
        pointpillar_->doinfer(points_data.data(), points_data.size() / 4, nms_pred);

        // 딥러닝 결과를 Tracking 노드(tracking2.cpp)가 받을 수 있게 포장
        vision_msgs::Detection3DArray bbox_array;
        bbox_array.header = msg->header;

        for (const auto& box : nms_pred) {
            vision_msgs::Detection3D det;
            det.header = msg->header;

            // 좌표
            det.bbox.center.position.x = box.x;
            det.bbox.center.position.y = box.y;
            det.bbox.center.position.z = box.z;

            // 박스 크기
            det.bbox.size.x = box.l; // 길이
            det.bbox.size.y = box.w; // 너비
            det.bbox.size.z = box.h; // 높이

            // 회전(Yaw 각도)을 쿼터니언으로 수동 변환 (Tracking 코드가 이해하도록)
            double yaw = box.rt;
            det.bbox.center.orientation.x = 0.0;
            det.bbox.center.orientation.y = 0.0;
            det.bbox.center.orientation.z = std::sin(yaw / 2.0);
            det.bbox.center.orientation.w = std::cos(yaw / 2.0);

            // 클래스 정보 (차량, 사람 등) 및 정확도(Score)
            vision_msgs::ObjectHypothesisWithPose hyp;
            hyp.id = box.id; 
            hyp.score = box.score;
            det.results.push_back(hyp);

            bbox_array.detections.push_back(det);
        }

        // Tracking 노드를 향해 발사!
        pub_bboxes_.publish(bbox_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gigacha_pointpillars_ros");
    PointPillarsROS node;
    ros::spin();
    return 0;
}