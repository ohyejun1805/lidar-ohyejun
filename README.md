# LiDAR Clustering and Tracking Package

LiDAR 데이터를 사용한 객체 클러스터링 및 트래킹 ROS 패키지입니다.

## 📋 개요

이 패키지는 Velodyne LiDAR 센서로부터 받은 Point Cloud 데이터를 처리하여:
1. **클러스터링**: 주변 객체들을 감지하고 바운딩 박스를 생성합니다.
2. **트래킹**: 칼만 필터와 헝가리안 알고리즘을 사용하여 여러 프레임에 걸쳐 객체를 추적합니다.

## 🚀 주요 기능

### 1. LiDAR Clustering (`lidar.cpp`)
- **Voxel Grid 필터링**: Point Cloud 데이터 다운샘플링
- **ROI (Region of Interest) 크롭**: 관심 영역만 추출
- **Euclidean 클러스터링**: 가까운 점들을 그룹화하여 객체 단위로 분리
- **바운딩 박스 생성**: 각 객체에 대한 3D 바운딩 박스 생성

### 2. Object Tracking (`tracking2.cpp`)
- **칼만 필터**: 객체의 위치와 속도를 추정하고 예측
- **헝가리안 알고리즘**: 최적의 객체-트랙 매칭 수행
- **Mahalanobis 거리**: 통계적 거리를 사용한 정확한 매칭
- **Cost Matrix**: 거리 기반 비용 행렬 생성

## 📦 의존성

### ROS 패키지
- `roscpp`
- `sensor_msgs`
- `geometry_msgs`
- `vision_msgs`
- `pcl_ros`
- `pcl_conversions`

### 시스템 라이브러리
- `libeigen3-dev` (Eigen3 라이브러리)
- `libpcl-all-dev` (PCL 라이브러리)

## 🔧 설치 및 빌드

### 1. 의존성 설치

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    ros-<distro>-pcl-ros \
    ros-<distro>-pcl-conversions \
    libeigen3-dev \
    libpcl-dev
```

> **참고**: `<distro>`를 사용 중인 ROS 배포판으로 변경하세요 (예: `melodic`, `noetic`, `humble`)

### 2. 워크스페이스 설정

```bash
# catkin 워크스페이스로 이동 (또는 새로 생성)
cd ~/catkin_ws/src

# 패키지 복사 또는 클론
git clone <your-repo-url> lidar-ohyejun
# 또는
cp -r /path/to/lidar-ohyejun .
```

### 3. 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🎮 실행 방법

### 1. LiDAR Clustering 노드 실행

```bash
rosrun lidar_ohyejun gigacha_lidar_clustering
```

**입력 토픽:**
- `/velodyne_points` (sensor_msgs/PointCloud2): LiDAR 원본 데이터

**출력 토픽:**
- `/gigacha/lidar/cloud_origin` (sensor_msgs/PointCloud2): 원본 Point Cloud
- `/gigacha/lidar/cloud_downsampled` (sensor_msgs/PointCloud2): 다운샘플링된 Point Cloud
- `/gigacha/lidar/cloud_roi` (sensor_msgs/PointCloud2): ROI 영역 Point Cloud
- `/gigacha/lidar/cloud_clustered` (sensor_msgs/PointCloud2): 클러스터링된 Point Cloud
- `/gigacha/lidar/bounding_boxes` (vision_msgs/Detection3DArray): 바운딩 박스 배열

### 2. Object Tracking 노드 실행

```bash
rosrun lidar_ohyejun gigacha_lidar_tracking
```

**입력 토픽:**
- `/gigacha/lidar/bounding_boxes` (vision_msgs/Detection3DArray): 클러스터링 노드의 바운딩 박스

**출력 토픽:**
- `/gigacha/lidar/tracked_objects` (vision_msgs/Detection3DArray): 트래킹된 객체 배열 (트랙 ID 포함)

## ⚙️ 파라미터 설정

### LiDAR Clustering 파라미터

노드 실행 전에 파라미터를 설정할 수 있습니다:

```bash
rosparam set /gigacha_lidar_clustering/voxel_size 0.1
rosparam set /gigacha_lidar_clustering/roi_min_x -10.0
rosparam set /gigacha_lidar_clustering/roi_max_x 30.0
rosparam set /gigacha_lidar_clustering/roi_min_y -10.0
rosparam set /gigacha_lidar_clustering/roi_max_y 10.0
rosparam set /gigacha_lidar_clustering/roi_min_z -1.8
rosparam set /gigacha_lidar_clustering/roi_max_z 0.3
rosparam set /gigacha_lidar_clustering/cluster_tolerance 0.4
rosparam set /gigacha_lidar_clustering/min_cluster_size 3
rosparam set /gigacha_lidar_clustering/max_cluster_size 6000
```

또는 launch 파일을 사용할 수도 있습니다:

```xml
<launch>
  <node name="gigacha_lidar_clustering" pkg="lidar_ohyejun" type="gigacha_lidar_clustering">
    <param name="voxel_size" value="0.1"/>
    <param name="roi_min_x" value="-10.0"/>
    <param name="roi_max_x" value="30.0"/>
    <!-- ... 기타 파라미터 ... -->
  </node>
</launch>
```

### Tracking 파라미터

```bash
rosparam set /gigacha_lidar_tracking/max_mahalanobis_distance 3.0
rosparam set /gigacha_lidar_tracking/max_disappeared_frames 5
rosparam set /gigacha_lidar_tracking/gating_threshold 9.0
```

## 📊 파라미터 설명

### Clustering 파라미터
- `voxel_size`: Voxel Grid 필터의 리프 크기 (미터)
- `roi_min_x/max_x`: ROI X축 범위
- `roi_min_y/max_y`: ROI Y축 범위
- `roi_min_z/max_z`: ROI Z축 범위
- `cluster_tolerance`: 클러스터링 허용 거리 (미터)
- `min_cluster_size`: 최소 클러스터 크기 (포인트 수)
- `max_cluster_size`: 최대 클러스터 크기 (포인트 수)

### Tracking 파라미터
- `max_mahalanobis_distance`: Mahalanobis 거리 임계값
- `max_disappeared_frames`: 객체가 사라진 후 유지할 프레임 수
- `gating_threshold`: 게이팅 임계값 (chi-square, 3 DOF, 99% 신뢰도)

## 🔍 시각화

RViz를 사용하여 결과를 시각화할 수 있습니다:

```bash
rosrun rviz rviz
```

RViz에서 다음 토픽들을 추가하세요:
- `/gigacha/lidar/cloud_clustered` (PointCloud2)
- `/gigacha/lidar/bounding_boxes` (Detection3DArray)
- `/gigacha/lidar/tracked_objects` (Detection3DArray)

## 📁 프로젝트 구조

```
lidar-ohyejun/
├── CMakeLists.txt          # 빌드 설정
├── package.xml             # ROS 패키지 메타데이터
├── README.md               # 이 파일
└── src/
    ├── lidar.cpp          # LiDAR 클러스터링 노드
    ├── tracking.cpp        # (미사용)
    └── tracking2.cpp       # 객체 트래킹 노드 (칼만 필터 + 헝가리안)
```

## 🛠️ 기술 스택

- **ROS**: 로봇 운영 체제
- **PCL (Point Cloud Library)**: Point Cloud 처리
- **Eigen3**: 선형 대수 연산 (칼만 필터)
- **C++11**: 프로그래밍 언어

## 📝 알고리즘 상세

### 칼만 필터
- 상태 벡터: `[x, y, z, vx, vy, vz]` (위치 + 속도)
- 등속 운동 모델 사용
- 예측 및 업데이트 단계를 통해 객체 위치와 속도 추정

### 헝가리안 알고리즘
- 최적 할당 문제 해결
- 전체 비용을 최소화하는 매칭 수행
- Munkres 알고리즘 구현

### Mahalanobis 거리
- 칼만 필터의 공분산 행렬을 고려한 통계적 거리
- 불확실성을 반영한 정확한 매칭

## 🐛 문제 해결

### 빌드 오류
- **Eigen3를 찾을 수 없음**: `sudo apt-get install libeigen3-dev`
- **PCL을 찾을 수 없음**: `sudo apt-get install libpcl-dev`
- **vision_msgs를 찾을 수 없음**: `sudo apt-get install ros-<distro>-vision-msgs`

### 실행 오류
- **토픽을 찾을 수 없음**: LiDAR 센서가 실행 중인지 확인
- **메모리 부족**: `voxel_size`를 늘려서 데이터 양 줄이기

## 📄 라이선스

MIT License

## 👤 작성자

GIGACHA Team

## 🙏 감사의 말

이 프로젝트는 ROS, PCL, Eigen3 오픈소스 라이브러리를 사용합니다.
