#include <ros/ros.h>
/* ros2 프로그래밍 기초 c++ 코드랑 ros2랑 통신하게 해주는 역할
노드 생성, 데이터 통신 밑에 있는 Publisher/Subscriber, 로그 출력, 시간 관리 같은 ros 기본기능 포함되어 있는 라이브러리
*/
#include <sensor_msgs/PointCloud2.h>
/* ros에서 센서 데이터를 주고받을 때 사용하는 표준 규격 헤더
lidar에서 나오는 3차원 point cloud 데이터를 담는 택배 박스 같은 것.
ros(메시지)랑 pcl(알고리즘) 사이의 다리 역할을 해줌.
*/
#include <limits>
/* c++ 표준 라이브러리 중 하나
int, float, double 같은 자료형이 가지는 가장 큰 값, 가장 작은 값 min, max을 알려줌.
밑에서 최소값/ 최대값 알고리즘에 초기화 용도로 사용됨.
max()랑 lowest() 뭔가 max랑 min 비교할 때 처음 들어오는 값이 무슨 값이든
그 값으로 초기화되도록 초기값을 제일 큰 수로 설정해야됨.
*/

// 아래 3개는 pcl의 핵심 파일들
#include <pcl_conversions/pcl_conversions.h>
/* ros 데이터 형식 sensor_msgs랑 pcl 라이브러리 데이터 형식인 PointCloud 사이를 연결
ros는 데이터를 직렬화해서 보냄. 네트워크 통신을 위해서
pcl 알고리즘은 c++ 객체 형태로 데이터가 필요함.
그래서 이 두개를 상호변환해주는 함수를 이 라이브러리에서 제공
이 코드에서 이 라이브러리는?
데이터를 받자마자, 보내기 직전에 사용됨
ROS -> PCL로 변환하거나 pcl::fromROSMsg
PCL -> ROS로 변환함 pcl::toROSMsg
*/
#include <pcl/point_cloud.h>
/*
pcl에서 Point cloud 전체를 담는 container 용기 클래스임.
vector랑 비슷한데 3D 포인트 처리하는데 특화된 멤버 함수와 속성들을 가지고 있음
이 클래스는 템플릿 클래스임.
어떤 종류의 점을 담을지에 따라 pcl::PointXYZ, pcl::PointXYZI 등으로 변함.
데이터를 담아두는 메인 메모리 공간을 생성할 때 사용
*/
#include <pcl/point_types.h>
/*
점 하나하나가 어떤 데이터 structure를 가질지 정의해 놓는 헤더
점 하나라도 데이터 구조가 다를 수 있다.
어떤 점은 XYZ 좌표만 있고, 어떤 점은 색상도 포함하고, 어떤 점은 반사 강도가 있다.
이런 구조체들을 미리 정의해둔 것임.
pcl::PointXYZ : 위치만 (x,y,z)
pcl::PointXYZI : 위치 + 반사강도 (+intensity)
pcl::PointXYZRGB : 위치 + 색상 (x,y,z,r,g,b)
lidar 에서는 XYZI 씀. 레이저가 반사되어서 돌아오는 세기 정보가 필요함
왜? 차선 인식이나 물체 재질 구분에 유용하기 때문에
*/
#include <pcl/filters/voxel_grid.h>
/*
복셀 그리드 필터
이 라이브러리는 다운샘플링 역할을 한다. 데이터 경량화. 점의 개수는 
수만 수십만 개의 점으로 이루어진 클라우드 데이터를 경량화시키는 역할을 한다.
3D 모든 점들을 없애고, 모양은 유지하고 무게중심에 해당하는 점으로 
군량화하는 역할을 합니다.
*/
#include <pcl/filters/crop_box.h>
/*
box 처리하는 느낌인 것 같은 직사각형 모양에 점들만 남기고, 그 외의 것들을 제거할 때 사용하는 라이브러리
x,y,z 축의 최소값min과 최대값max를 지정하여 박스를 만들고, 
단순한 범위 지정이 아니고 박스를 회전시키고나 이동시켜서 더 정교하게 잘라낼 수 있다.
→ 내 차체의 점들을 지워줌.
→ 로보틱스에서 팔이 작업할 때 작업 공간인 테이블 위 특정 공간만 남기고 나머지 배경은 다 날려버림.
위에껀 청소하는 거고 밑에꺼는 의미 있는 물체로 구분하는 단계입니다. 뭐 나무랑, 자동차, 사람 등으로
*/
#include <pcl/kdtree/kdtree.h>
/*
내 주변에 제일 가까이 있는 수만 개의 점들은 엄청나게 빨리 찾게 해주는 자료구조입니다.
kd-tree는 도서관처럼 트리 구조로 뭐 총류→ 300번대 → 320번대 이런식으로 빨리 찾을 수 있게 해줌.
*/
#include <pcl/segmentation/extract_clusters.h>
/*
클러스터 추출 라이브러리 군집화 하는거.
가까이 있는 점들끼리 같은 색깔로 칠해주는 알고리즘임.
작동원리 : 점 하나 찍고, kd-tree 한테 나한테 반경 몇 cm 안에 있는 점들 다 찾아줘. 하고 그걸 그룹으로 묶고, 
새로 묶인 점들 기준으로 주변을 또 찾는다. 더 이상 연결되는 점이 없으면 하나의 덩어리 (Cluster) 완성.
처음엔 그냥 점들의 구름(Point Cloud) 같지만, 이 과정을 거치면 자동차, 보행자, 가로수처럼 개별 객체로 나뉩니다.
아래 헤더 파일들은 ROS의 표준 메시지 패키지인 vision_msgs 라이브러리입니다.
PCL이 데이터를 가공하는 도구였다면, 이 라이브러리는 가공된 결과를 다른 프로그램(노드)로 배송하기 위한 규격 상자입니다.
PCL로 물체를 찾으면 다른 파트 (주행 제어, 경로 계획) 등을 알려줄때 사용하는 표준 통신 프로토콜이라고 보시면 됩니다.
*/

#include <vision_msgs/Detection3DArray.h>
/*
전방에 발견된 물체 리스트 모으는거.
한 프레임에 발견된 모든 3D 물체들의 목록을 담는 메시지. 내부에 Detection3D라는 메시지를 여러 개 담을 수 있는 배열 형태임.
예시 : 내 차 앞에 승용차 1대, 사람 2명, 가로수 1그루가 있어 라는 전체 정보를 담아 보내는 라이브러리
*/
#include <vision_msgs/ObjectHypothesisWithPose.h>
/*
이 물체가 뭐고, 어디에 있는지에 대한  추정값. 개별 물체 하나 당 구체적인 정보를 담음.
Hypothesis(가설/추정) : 이 물체는 95% 확률로 사람입니다.
With Pose(위치/자세) : 그리고 내 차 기준으로 어디 위치에 서 있습니다.
위에 딥러닝이나 클러스터링 결과로 나온 class id(물체 종류)와 score(신뢰도), 그리고 Pose(좌표)를 여기에 채워 넣습니다.
*/

using PointT = pcl::PointXYZI;
// XYZ 좌표에 intensity(강도)까지 변수로 가지는 Point 사용 (intensity는 차선 식별에 도움을 주는 변수)

class GigachaLidarClustering//클래스 선언
{
private:
    ros::NodeHandle nh_;
    /*
    → 노드 핸들, 이 프로그램이 ROS 시스템과 소통(통신, 파라미터 로드 등) 하기 위한 핵심 매니저 
    handle(매니저) : 우리가 짠 C++ 코드(gigachaLidarClustering class) 가 ROS에 접속하기 위한 공식 통로
    ROS 네트워크에 데이터를 보내기 위해서는 NodeHandle이라는 대리인을 통해야 한다.
    nh_ 생성자 호출시 ROS 본부 roscore에 gigachaLidarClustering이라는 새로운 노드를 등록시킴.
    이걸로 publisher, subscriber 생성시킴.
    publisher : 내가 데이터 보내고 싶을 때 (advertise), 직접 보내지 않고, nh_ 통해서 cloud_cluster의 publisher 객체를 줌.
    subscriber : 내가 데이터를 받고 싶을 때 lidar_topic이라는 데이터 달라고 하면, nh_에서 데이터가 오면 callback 해줌.
    기술적으로 NodeHandle은 API 게이트웨이로, RAII 패턴을 따르는 인터페이스 객체임. 
    1. 네임스페이스 관리 (namespace)
    nh_는 이 노드가 현재 /root 에 있는지, /robot1/sensor 같은 하위 그룹에 있는지 경로를 관리합니다.
    마치 파일 시스템의 handle 과 같음.
    2. Resource 관리
    nh_.advertise<…>()나 nh_.subscribe(…) 함수들은 nh_ 객체의 멤버 함수 입니다. 
    ROS 통신은 모두 이 객체를 거져야만 접근할 수 있습니다. 
    3. 수명관리 (lifecycle)
    nh_ 객체가 소멸되면(클래스 파괴) ROS master에게 shutdown이라고 자동으로 통보하고 연결을 끊음. (소멸자에서 처리)
    */
    ros::Subscriber lidar_sub_;
    /*
    수신자임. 라이다 센서에서 나오는 원본 데이터 PointCloud2를 받아오는 역할을 함.
    */
    ros::Publisher cloud_origin_pub_;
    /*
    송신자(원본), 시각화 확인용으로 원본 점구름을 다시 쏘아주는 역할을 함. 변수 이름에 의미가 다 있음.
    */
    ros::Publisher cloud_down_pub_;
    //송신자인데 downsampling, voxelgrid을 거쳐 점 개수가 줄어든 데이터 보내는 것.
    ros::Publisher cloud_crop_pub_;
    //송신자(크롭) Cropbox를 거쳐서 관심 영역만 남은 데이터를 내보냄.
    ros::Publisher cloud_cluster_pub_;
    //송신자(군집화), 물체별로 색깔이 칠해진 최종 점구름 데이터를 보냅니다. 
    ros::Publisher bbox_pub_;
    //ROS의 publisher 변수임. 바운딩 박스 관련 메시지를 발행하는데 사용함.
    float voxel_size_;//3D 픽셀 크기 변수
    float roi_min_x_, roi_max_x_;
    float roi_min_y_, roi_max_y_;
    float roi_min_z_, roi_max_z_;//roi는 region of interest 관심 영역.
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

    bool publish_origin_;
    bool publish_down_;
    bool publish_crop_;
    bool publish_clustered_;
    //각 단계별 데이터 publish 여부를 제어하는 bool 변수.
public:
    GigachaLidarClustering() : nh_("~")
    //기가차라이다클러스팅 생성자가 호출되면, nh_("~") : nh_를 "~" 네임스페이스로 초기화 private nodehandle 객체 생성
    //왜 private로 만드냐? 코드 재사용성, 파라미터 충돌 방지
    //예를 들어서 라이더 두 개 쓰면, 파라미터 이름이 똑같으면 충돌나니까. private nodehandle 써서
    //똑같은 소스 코드 써도, 서로 다른 설정값을 가질 수 있음.
    {
        ROS_INFO("GIGACHA LiDAR Clustering Node1 Starting...");
        /*
        nh_.param 함수는 nh_.param<타입>("파라미터 이름",변수,기본값);의 기본 구조를 가짐.
        nh_는 Nodehandle 객체고, param은 파라미터를 읽어오는 템플릿 함수.
        <타입>은 템플릿 타입, "파라미터 이름"은 파라미터 서버에서 찾을 이름이고,
        변수는 값을 저장할 변수, true는 기본값값
        */
        nh_.param<float>("voxel_size", voxel_size_, 0.1f);
        nh_.param<float>("roi_min_x", roi_min_x_, -10.0f);
        nh_.param<float>("roi_max_x", roi_max_x_, 30.0f);
        nh_.param<float>("roi_min_y", roi_min_y_, -10.0f);
        nh_.param<float>("roi_max_y", roi_max_y_, 10.0f);
        nh_.param<float>("roi_min_z", roi_min_z_, -1.8f);
        nh_.param<float>("roi_max_z", roi_max_z_, 0.3f);
        nh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.4f);
        nh_.param<int>("min_cluster_size", min_cluster_size_, 3);
        nh_.param<int>("max_cluster_size", max_cluster_size_, 6000);
        //ROS 파라미터에서 값을 읽고, 없으면 기본값을 넣는다.

        nh_.param<bool>("publish_origin", publish_origin_, true);
        //파라미터 서버에 publish_origin이 있으면, true 또는 false를 publish_origin_ 변수에 저장하고
        //없으면 그냥 기본값 true 저장하는거임.
        nh_.param<bool>("publish_down", publish_down_, true);
        nh_.param<bool>("publish_crop", publish_crop_, true);
        nh_.param<bool>("publish_clustered", publish_clustered_, true);

        //얘네 다 publisher 임.
        cloud_origin_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_origin", 1);
        cloud_down_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_downsampled", 1);
        cloud_crop_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_roi", 1);
        cloud_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gigacha/lidar/cloud_clustered", 1);
        /*
        advertise라는 함수인데 라이다 점군 데이터를 타입으로 하는 함수다. 
        ""안에 있는 건 토픽 이름이라고 하네요. ros는 모든 통신을 이름으로 구분한다. 이를 토픽이라고 하고,
        cloud_origin_pub가 데이터를 보내는데 그 토픽 이름이 "/gigacha/lidar/cloud_origin이야
        publisher에서 이렇게 토픽 이름 지정하는 거고, subscriber는 반드시 이 토픽이름과 똑같이 적어야만 데이터를 받을 수있음.
        뒤에 1은 큐 사이즈이고, 데이터를 몇개까지 저장해 둘 것인가, 라이다는 0.1초마다 새로운 데이터들을 왕창 보내기 때문에
        컴퓨터가 조금이라도 랙 걸리면, 가장 최신 거 1개만 남기고 나머진 다 갖다 버린다는 것. 가장 최신 데이터만을 가짐!
        모든 자율주행과 로봇 제어에서 최근 데이터를 굉장히 중요하게 생각하기 때문에 큐 사이즈를 대부분 1로 지정함.
        */
        bbox_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/bounding_boxes", 1);
        //Detection3DArray : 중심 좌표, 크기 (가로세로높이), 회전 정보를 보내는 박스 배열. 주변에 장애물이 최소 1개 이상일테니까
        //그 장애물 배열들을 싹 publish 하는거임.
        lidar_sub_ = nh_.subscribe("/velodyne_points", 1, &GigachaLidarClustering::lidarCallback, this);
        /*
        lidar_sub_은 수신받는다(subscribe),"/velodyne_points"라는 데이터만 최신거 1개,
        &GigachaLidarClustering::lidarCallback이게 콜백함수로 subscribe에서 가장 중요한 것,
        나중에 이 라이다 데이터가 도착하면, ROS가 자동으로 이 lidarCallback 함수를 실행시켜줌.
        this는 이 callback 함수가 지금 이 class 안에 구현이 되어있는 클래스 멤버함수라는 것.
        */
        ROS_INFO("GIGACHA LiDAR Clustering Node Initialized");
        ROS_INFO("Parameters:");
        ROS_INFO("  Voxel Size: %.2f", voxel_size_);
        ROS_INFO("  ROI: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]",
                 roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_);
        ROS_INFO("  Clustering: tolerance=%.2f, size=[%d,%d]",
                 cluster_tolerance_, min_cluster_size_, max_cluster_size_);
        /*
        ROS_INFO는 c언어 printf같은 느낌인데, 훨씬 기능 많음.
        ROS_INFO는 타임스탬프가 자동으로 찍힘. 언제 메시지가 발생하였는지 시간 정보를 자동으로 앞에 붙여줌.
        ROS_INFO는 흰색 글씨로 나오고, ROS_WARN은 노란색으로 '경고!'라고 띄워주고, ROS_ERROR는 빨간색으로 '에러!'라고 띄워줌.
        내 로봇이 멀리 떨어져있어도, 네트워크로 내 노트북으로 로그가 날라옴. (rqt_console 툴로 확인)
        
        지금 이 ROS_INFO는 파라미터 이렇게 설정되었다는 알림 정도.
        */
    }

    ~GigachaLidarClustering()//소멸자
    {
        ROS_INFO("GIGACHA LiDAR Clustering Node Terminated");
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    //앞에 const는 이제 데이터 보여주고 읽기만 한다음에 수정은 안된다는거 배운거지.
    //sensor_msgs::PointCloud2는 라이다 점 포멧, 거대한 데이터 덩어리
    //ConstPtr이 스마트 포인터, 왜 스마트 포인터냐면 new, delete 안해도 알아서 메모리 해제해줌.
    //&도 포인터 자체도 복사 안하고 바로 접근해서 속도를 극대화 하는거 (call by reference)
    {
        pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_down(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
        //new니까 heap 영역에 메모리 잡음.
        //pointT 타입 원본, 다운, 크롭 동적 객체 생성
        //Ptr(스마트 포인터) : 포인터 쓰는 곳 없으면 알아서 delete 호출해서 메모리 해제함.(참조 카운팅(reference counting))
        //힙 메모리에 거대한 PointCloud 객체를 하나 만듬.
        //그리고 그 주소값을 스마트 포인터 cloud_origin로 연결함.
        pcl::fromROSMsg(*msg, *cloud_origin);
        /*fromROSMsg는 pcl 라이브러리 전역 함수
        입력 : *msg는 lidarCallback 함수 변수이고, ROS의 PointCloud2 메세지.
        출력 : *cloud_origin는 PointCloud<PointT>의 객체 이는 xyz 멤버 변수가 정확히 정의된 구조체들의 벡터
        ROS 데이터를 PCL 구조체로 변환해줌.
        */

        if (cloud_origin->empty())
        {
            ROS_WARN("Empty point cloud received");
            return;
        }//cloud_origin 데이터 비워져있으면 warning 띄움.

        pcl::VoxelGrid<PointT> voxel_filter;//voxel_filter는 약간 데이터 줄이는 용도인듯.
        voxel_filter.setInputCloud(cloud_origin);//cloud_origin 데이터 넣고,
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);//LeafSize 설정
        voxel_filter.filter(*cloud_down);//filter해서 cloud_down시킴.

        pcl::CropBox<PointT> crop_filter;//CropBox라는 필터 객체 생성
        crop_filter.setInputCloud(cloud_down);//voxel_filter 거친 데이터 넣음
        crop_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1.0f));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1.0f));
        //관심 영역 직육면체의 최소값과 최대값 설정
        crop_filter.filter(*cloud_crop);
        //이 관심 영역 안에 있는 점들만 cloud_crop에 넣음.

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_crop);
        /*
        kdTree는 K-Dimensional Tree의 약자로, 3D 공간을 반으로 쪼개고, 쪼개서, 데이터를 트리구조로 정리해둔 자료구조
        tree라는 스마트 포인터를 생성하고, cloud_crop 데이터를 넣음.
        3D 공간에서 특정 점이 주어졌을 때, 그 점과 가장 가까운 점들을 빠르게 찾아주는 역할을 함.
        */

        std::vector<pcl::PointIndices> cluster_indices;//군집화한 점들의 인덱스를 저장할 벡터

        pcl::EuclideanClusterExtraction<PointT> clustering;
        //유클리드 군집화 객체 생성
        //처리하지 않은 점 하나 선택하고 그 점과 일정 거리 이내에 있는 점들을 모두 군집화
        clustering.setInputCloud(cloud_crop);//자른 cloud_crop 데이터를 넣음
        clustering.setClusterTolerance(cluster_tolerance_);//군집화 허용 오차 설정
        clustering.setMinClusterSize(min_cluster_size_);
        //최소 군집 크기 설정
        //점이 너무 조금 있으면 물체로 인식하지 않도록 설정(노이즈)
        clustering.setMaxClusterSize(max_cluster_size_);
        //최대 군집 크기 설정
        //점이 너무 많으면 물체로 인식하지 않도록 설정(땅)
        clustering.setSearchMethod(tree);//위에서 만든 kd-tree를 검색 방법으로 설정
        clustering.extract(cluster_indices);//실행해서 위에 만든 cluster_indices 벡터에 결과 저장

        pcl::PointCloud<PointT>::Ptr cloud_clustered(new pcl::PointCloud<PointT>);
        //동적 객체로 힙 영역에 군집화된 점들을 저장할 PointCloud 객체 생성
        cloud_clustered->header = cloud_crop->header;
        /*
        헤더 정보 다 복사해줌.
        header에 있는 정보 : seq(메시지 번호), stamp(타임스탬프), frame_id(좌표계 이름)
        */
        cloud_clustered->is_dense = false;
        //이 데이터가 조밀하지 않다는 것 
        //즉, 일부 점들이 NaN 값 같은 쓰레기값을 가질 수 있다는 의미

        vision_msgs::Detection3DArray detection_array;
        //발견된 물체들을 담을 Detection3DArray 메시지 객체 생성
        detection_array.header.stamp = msg->header.stamp;
        //원본 메시지의 타임스탬프 복사
        detection_array.header.frame_id = msg->header.frame_id;
        //공간 좌표계 정보 복사. Velodyne 좌표계라는 의미

        //찾아낸 덩어리들 하나하나 for문으로 처리
        int cluster_id = 0;
        for (const auto &indices : cluster_indices)
        /*
        extract에서 채워진 cluster_indices 벡터에서 하나씩 꺼내서
        indices 라는 변수에 저장하면서 반복문 실행
        */
        {
            if (indices.indices.empty())//군집화된 인덱스가 비어있으면
            {
                cluster_id++;//다음 군집으로 넘어감
                continue;
            }

            //어떤값이 들어와도 min또는 max가 되도록 초기값을 양극단으로 설정
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();

            for (const auto &idx : indices.indices)//군집화된 점들의 인덱스 하나씩 꺼내서
            {
                const auto &p = cloud_crop->points[idx];//cloud_crop에서 해당 인덱스의 점을 p에 저장
                min_x = std::min(min_x, p.x);
                max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y);
                max_y = std::max(max_y, p.y);
                min_z = std::min(min_z, p.z);
                max_z = std::max(max_z, p.z);//각 축별 최소값과 최대값 갱신

                PointT q = p;//원본 점은 바꾸면 안되니까 복사본 q 생성
                q.intensity = static_cast<float>(cluster_id);
                //intensity 값을 군집 아이디로 설정해서 색깔 다르게 만듬.
                cloud_clustered->points.push_back(q);
                //cloud_clustered에 q점 추가
            }

            float size_x = max_x - min_x;
            float size_y = max_y - min_y;
            float size_z = max_z - min_z;
            //군집의 size 계산
            //size 계산해서 너무 작거나 크면 무시

            if (size_y < 0.3f || size_y > 1.5f || size_z > 2.0f)
            //너무 thin 하거나 너무 fat 하거나 너무 높으면
            {
                cluster_id++;
                continue;//다음
            }

            vision_msgs::Detection3D detection;//detection3D 메시지 객체 생성
            detection.header = detection_array.header;//헤더 정보 복사

            detection.bbox.center.position.x = (min_x + max_x) * 0.5f;
            detection.bbox.center.position.y = (min_y + max_y) * 0.5f;
            detection.bbox.center.position.z = (min_z + max_z) * 0.5f;
            //박스의 중심 좌표 계산
            detection.bbox.center.orientation.w = 1.0;
            //회전 정보 Quaternion으로 표현, 회전이 없으면 w=1.0, x=y=z=0.0
            //근데 왜 회전 정보가 없지? 계산 속도 때문에 생략한 듯. 
            //PCA(주성분 분석)으로 회전 정보도 구할 수 있지만 계산량이 많아짐.
            detection.bbox.size.x = size_x;
            detection.bbox.size.y = size_y * 1.2f;
            detection.bbox.size.z = size_z * 1.5f;
            //y축과 z축 크기는 살짝 여유를 줌.

            //물체 클래스와 신뢰도 점수 설정
            vision_msgs::ObjectHypothesisWithPose hypothesis;
            //이 물체가 뭔지 추측할 객체 생성
            hypothesis.id = 0;//점 밖에 없으니까 0으로 설정
            hypothesis.score = 1.0;//신뢰도 100% 설정. 라이다에 부딪힌 점들이니까 무조건 100프로 신뢰하는것.
            detection.results.push_back(hypothesis);//이 추측은 detection results에 추가

            detection_array.detections.push_back(detection);
            //여태까지 만든 detection 객체를 detection_array에 추가

            cluster_id++;
        }

        if (publish_origin_)//만약 원본 데이터를 publish 하기로 설정되어 있으면
        {
            sensor_msgs::PointCloud2 out;//빈 ROS 메시지 객체 생성
            pcl::toROSMsg(*cloud_origin, out);//PCL PointCloud를 ROS PointCloud2 메시지로 변환
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = msg->header.frame_id;//헤더 정보 복사
            cloud_origin_pub_.publish(out);//원본 점구름 데이터 publish
        }

        if (publish_down_)//만약 다운샘플된 데이터를 publish 하기로 설정되어 있으면
        {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_down, out);
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = msg->header.frame_id;
            cloud_down_pub_.publish(out);//똑같은 방식으로 publish
        }

        if (publish_crop_)//만약 크롭된 데이터를 publish 하기로 설정되어 있으면
        {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_crop, out);
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = msg->header.frame_id;
            cloud_crop_pub_.publish(out);//똑같은 방식으로 publish
        }

        if (publish_clustered_)//만약 군집화된 데이터를 publish 하기로 설정되어 있으면
        {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cloud_clustered, out);
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = msg->header.frame_id;
            cloud_cluster_pub_.publish(out);//똑같은 방식으로 publish
        }

        bbox_pub_.publish(detection_array);
        //detection_array 메시지를 /gigacha/lidar/bounding_boxes 토픽으로 publish

        //디버그 로그 출력
        ROS_DEBUG("origin=%zu down=%zu crop=%zu clusters=%zu det=%zu",
                  cloud_origin->size(),
                  cloud_down->size(),
                  cloud_crop->size(),
                  cluster_indices.size(),
                  detection_array.detections.size());
                  //내 데이터가 몇개인지 출력
                  //어디가 병목인지 파악하는데 도움됨.
    }//lidarCallback 함수 끝
};

int main(int argc, char **argv)//argc는 인자 개수, argv는 인자 값들(문자열 배열 포인터)
{
    ros::init(argc, argv, "gigacha_lidar_clustering");
    //노드 이름 설정
    //사용자가 터미널에 입력한 인자값들을 ROS 시스템에 전달
    GigachaLidarClustering node;
    //객체 생성하고, 생성자 호출됨.
    //생성자 안에서 파라미터(nh_.param) 읽고, publisher, subscriber 생성됨.
    ros::spin();
    //콜백 함수 대기 상태로 진입
    //무한 루프 돌면서 콜백 함수가 호출되기를 기다림.
    return 0;
}