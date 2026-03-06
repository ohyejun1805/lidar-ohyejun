#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cuda_runtime.h>  // [★추가] CUDA 함수들을 위해 필수!

// 우리가 찾은 진짜 헤더 파일
#include "pointpillar.hpp" 

class PointPillarsROS {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Publisher pub_bboxes_;
    
    // [★수정] NVIDIA 규격: pointpillar::lidar::Core 클래스 사용
    std::shared_ptr<pointpillar::lidar::Core> pointpillar_;
    cudaStream_t stream_;

public:
    PointPillarsROS() {
        // 엔진 경로 설정
        std::string engine_path = "/root/workspace/CUDA-PointPillars/model/pointpillar.plan";
        
        // CUDA 스트림 생성
        cudaStreamCreate(&stream_);

        // [★수정] NVIDIA 규격: CoreParameter 설정 및 create_core 호출
        pointpillar::lidar::CoreParameter param;
        param.lidar_model = engine_path;
        // 기본 파라미터들은 헤더에서 정의된 기본값을 따릅니다.

        pointpillar_ = pointpillar::lidar::create_core(param);
        
        sub_lidar_ = nh_.subscribe("/velodyne_points", 1, &PointPillarsROS::lidarCallback, this);
        pub_bboxes_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        
        ROS_INFO("🔥🔥 GIGACHA PointPillars (NVIDIA Core) Engine Started! 🔥🔥");
    }

    ~PointPillarsROS() {
        cudaStreamDestroy(stream_);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        std::vector<float> points_data;
        points_data.reserve(pcl_cloud->points.size() * 4);
        for (const auto& pt : pcl_cloud->points) {
            points_data.push_back(pt.x);
            points_data.push_back(pt.y);
            points_data.push_back(pt.z);
            points_data.push_back(pt.intensity);
        }

        // [★수정] NVIDIA 규격: doinfer 대신 forward 함수 사용
        auto nms_pred = pointpillar_->forward(points_data.data(), points_data.size() / 4, stream_);

        vision_msgs::Detection3DArray bbox_array;
        bbox_array.header = msg->header;

        // [★수정] Bndbox 대신 pointpillar::lidar::BoundingBox 사용
        for (const auto& box : nms_pred) {
            vision_msgs::Detection3D det;
            det.header = msg->header;

            det.bbox.center.position.x = box.x;
            det.bbox.center.position.y = box.y;
            det.bbox.center.position.z = box.z;
            det.bbox.size.x = box.l;
            det.bbox.size.y = box.w;
            det.bbox.size.z = box.h;

            double yaw = box.rt;
            det.bbox.center.orientation.z = std::sin(yaw / 2.0);
            det.bbox.center.orientation.w = std::cos(yaw / 2.0);

            vision_msgs::ObjectHypothesisWithPose hyp;
            hyp.id = box.id; 
            hyp.score = box.score;
            det.results.push_back(hyp);

            bbox_array.detections.push_back(det);
        }
        pub_bboxes_.publish(bbox_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gigacha_pointpillars_ros");
    PointPillarsROS node;
    ros::spin();
    return 0;
}