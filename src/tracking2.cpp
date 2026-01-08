#include <ros/ros.h>
/* 
ROS 기본 라이브러리
ROS 노드 생성, Publisher/Subscriber 통신, 로그 출력, 시간 관리 등 ROS의 핵심 기능을 제공하는 라이브러리
*/
#include <vision_msgs/Detection3DArray.h>
/* 
vision_msgs 패키지의 Detection3DArray 메시지 타입
한 프레임에 발견된 모든 3D 객체들의 목록을 담는 배열 형태의 메시지
lidar.cpp에서 생성한 바운딩 박스들을 받아오는 데 사용됨
*/
#include <vision_msgs/Detection3D.h>
/* 
vision_msgs 패키지의 Detection3D 메시지 타입
개별 3D 객체 하나의 정보를 담는 메시지 (위치, 크기, 회전 정보 등)
*/
#include <geometry_msgs/Point.h>
/*
geometry_msgs 패키지의 Point 메시지 타입
3D 공간의 한 점을 나타내는 메시지 (x, y, z 좌표)
객체의 중심점이나 위치를 표현할 때 사용 (lidar에서 썼던 거 그대로 사용)
*/
#include <limits>
/* 
C++ 표준 라이브러리
자료형의 최대값, 최소값을 알려주는 헤더
초기화할 때 양극단 값을 설정하는 데 사용
*/
#include <vector>
#include <map>
/* 
C++ 표준 라이브러리의 맵 (키-값 쌍)
데이터를 키로 빠르게 찾을 수 있는 자료구조
*/
#include <cmath>
/* 
C++ 표준 라이브러리의 수학 함수들
sqrt, pow 등 수학 계산에 사용
Mahalanobis 거리 계산 시 필요
*/
#include <algorithm>
#include <Eigen/Dense>
/* 
Eigen 라이브러리의 Dense 행렬 연산
칼만 필터에서 행렬 연산을 수행하기 위해 필요
VectorXd (동적 크기 벡터), MatrixXd (동적 크기 행렬) 등을 제공
*/


/*
이 프로그램은 lidar.cpp에서 생성된 바운딩 박스들을 입력으로 받아서,
여러 프레임에 걸쳐 같은 객체를 추적하는 역할을 합니다.

사용 알고리즘:
1. 칼만 필터: 객체의 위치와 속도를 추정하고 다음 위치를 예측
2. 헝가리안 알고리즘: 여러 객체와 트랙을 최적으로 매칭
3. Mahalanobis 거리: 칼만 필터의 불확실성을 고려한 통계적 거리
*/

// 칼만 필터 클래스
// 칼만 필터는 노이즈가 있는 측정값으로부터 객체의 실제 상태(위치, 속도)를 추정하는 알고리즘입니다.
// 예측 단계와 업데이트 단계를 반복하여 점점 더 정확한 상태를 추정합니다.
class KalmanFilter3D
{
private:
    // 상태 벡터: [x, y, z, vx, vy, vz]
    // 객체의 위치(x, y, z)와 속도(vx, vy, vz)를 하나의 벡터로 표현
    Eigen::VectorXd state_;           
    //현재 추정된 객체의 상태를 저장
    // 6x1 상태 벡터 (위치 3개 + 속도 3개)

    Eigen::MatrixXd P_;               
    // 6x6 공분산 행렬
    /* 상태 추정의 불확실성을 나타내는 행렬
    P_가 크면 = 불확실성이 크다 = 예측이 부정확하다
    P_가 작으면 = 불확실성이 작다 = 예측이 정확하다
    칼만 필터가 작동하면서 P_가 점점 작아져서 더 정확해짐
    */
    Eigen::MatrixXd F_;               
    // 6x6 상태 전이 행렬
    /* 한 프레임에서 다음 프레임으로 상태가 어떻게 변하는지 나타내는 행렬
    등속 운동 모델을 사용: 위치 = 이전 위치 + 속도 * 시간
    F_를 사용해서 "다음 프레임에 객체가 어디 있을지" 예측
    */
    Eigen::MatrixXd H_;               
    // 3x6 관측 행렬 (위치만 관측)
    /* 실제로 측정할 수 있는 것과 상태 벡터의 관계를 나타내는 행렬
    LiDAR는 위치(x, y, z)만 측정할 수 있고, 속도는 직접 측정 불가
    그래서 H_는 위치 3개만 추출하도록 설정 (상태 벡터 6개 중 앞에 거 xyz 3개만)
    */
    Eigen::MatrixXd Q_;               
    // 6x6 프로세스 노이즈
    /* 예측 과정에서 발생하는 노이즈 (모델의 불완전함)
    예를 들어, 객체가 가속하거나 방향을 바꿀 수 있는데 우리 모델은 등속 운동만 가정
    이런 차이를 Q_로 표현
    Q_가 크면 = 예측을 덜 신뢰 = 측정값을 더 많이 반영
    Q_가 작으면 = 예측을 더 신뢰 = 측정값을 덜 반영
    */
    Eigen::MatrixXd R_;               
    // 3x3 관측 노이즈
    /* 실제 측정값의 노이즈 (센서의 불확실성)
    LiDAR도 완벽하지 않아서 측정 오차가 있음
    R_가 크면 = 측정값을 덜 신뢰 = 예측을 더 많이 반영
    R_가 작으면 = 측정값을 더 신뢰 = 예측을 덜 반영
    */
    bool initialized_;                
    // 칼만 필터가 초기화되었는지 여부

public:
    KalmanFilter3D() : initialized_(false)
    /* 생성자: 칼만 필터 객체를 만들 때 초기 설정을 수행
    이 칼만필터 생성자가 호출되면, 멤버 변수 initialized_를 false로 초기화
    */
    {
        state_ = Eigen::VectorXd::Zero(6);
        /* 상태 벡터를 6개 요소 모두 0으로 초기화
        Zero(6)는 크기가 6인 벡터를 만들고 모든 값을 0으로 설정
        아직 아무것도 모르니까 0으로 시작
        */
        P_을 = Eigen::MatrixXd::Identity(6, 6) * 1000.0;  // 초기 불확실성 큼
        /* 공분산 행렬을 단위 행렬 * 1000으로 초기화
        Identity(6, 6)는 대각선이 1이고 나머지가 0인 6x6 행렬
        * 1000을 해서 대각선 값이 1000이 됨 = 불확실성이 매우 크다는 의미
        처음에는 아무것도 모르니까 불확실성 크게 설정
        */
        
        // 상태 전이 행렬 (등속 운동 모델)
        F_ = Eigen::MatrixXd::Identity(6, 6);
        /* 상태 전이 행렬을 단위 행렬로 초기화
        나중에 predict() 함수에서 시간 dt에 따라 업데이트됨
        Identity로 시작해서 나중에 필요한 부분만 수정
        */
        
        // 관측 행렬 (위치만 관측)
        H_ = Eigen::MatrixXd::Zero(3, 6);
        /* 관측 행렬을 3x6 크기의 0 행렬로 초기화
        3행 = x, y, z 위치 3개
        6열 = 상태 벡터 6개 (x, y, z, vx, vy, vz)
        */
        H_(0, 0) = 1.0;  
        H_(1, 1) = 1.0;  
        H_(2, 2) = 1.0;  
        //x, y, z 위치 3개만 관측하도록 설정
        
        // 프로세스 노이즈 (작을수록 예측을 더 신뢰)
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        /* 프로세스 노이즈를 단위 행렬 * 0.1로 설정
        위치에 대한 노이즈는 0.1로 작게 설정 (위치는 예측이 잘 됨)
        */
        Q_(3, 3) = 0.5;  // 속도 노이즈
        /* Q_의 (3, 3) 위치에 0.5 설정
        이건 x 방향 속도에 대한 노이즈
        속도는 위치보다 예측하기 어려우니까 노이즈를 더 크게 설정
        */
        Q_(4, 4) = 0.5;
        /* y 방향 속도에 대한 노이즈도 0.5로 설정
        */
        Q_(5, 5) = 0.5;
        /* z 방향 속도에 대한 노이즈도 0.5로 설정
        */
        
        // 관측 노이즈 (LiDAR 측정 불확실성)
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.3;
        /* 관측 노이즈를 단위 행렬 * 0.3으로 설정
        LiDAR의 측정 오차를 나타냄
        0.3은 약 30cm 정도의 측정 불확실성을 의미
        */
    }
    
    void init(const geometry_msgs::Point& position)
    /* 칼만 필터를 초기화하는 함수
    첫 번째 측정값(위치)을 받아서 칼만 필터를 시작
    */
    {
        state_(0) = position.x;
        /* 상태 벡터의 0번째 요소(x 위치)를 측정값으로 설정
        */
        state_(1) = position.y;
        /* 상태 벡터의 1번째 요소(y 위치)를 측정값으로 설정
        */
        state_(2) = position.z;
        /* 상태 벡터의 2번째 요소(z 위치)를 측정값으로 설정
        */
        state_(3) = 0.0;
        //처음에는 속도를 모르니까 0으로 시작
        state_(4) = 0.0;
        state_(5) = 0.0;

        
        P_ = Eigen::MatrixXd::Identity(6, 6) * 100.0;
        /* 공분산 행렬을 단위 행렬 * 100으로 설정
        생성자에서 1000이었는데 여기서는 100으로 줄임
        첫 측정값이 들어왔으니 불확실성을 조금 줄임
        */
        initialized_ = true;
        /* 
        초기화가 완료되었다는 의미.
        아래 predict()와 update() 함수에서 initialized_가 true인지 확인해서 초기화 여부를 판단.
        */
    }
    
    void predict(double dt)
    /* 예측 단계: 다음 프레임에서 객체가 어디 있을지 예측
    dt는 이전 프레임과 현재 프레임 사이의 시간 차이 (초 단위)
    */
    {
        if (!initialized_) return;
        /* 초기화되지 않았으면 아무것도 하지 않고 리턴
        초기화 전에는 예측할 수 없음
        */
        
        // 상태 전이 행렬 업데이트 (시간에 따라)
        F_(0, 3) = dt;  // x = x + vx*dt
        /* 상태 전이 행렬의 (0, 3) 위치에 dt 설정
        이건 "x 위치 = 이전 x 위치 + x 속도 * 시간"을 의미
        F_ 행렬을 곱하면 자동으로 이 공식이 적용됨
        */
        F_(1, 4) = dt;  // y = y + vy*dt
        /* y 위치 = 이전 y 위치 + y 속도 * 시간
        */
        F_(2, 5) = dt;  // z = z + vz*dt
        /* z 위치 = 이전 z 위치 + z 속도 * 시간
        */
        
        // 예측 단계
        state_ = F_ * state_;
        /* 상태 전이 행렬을 상태 벡터에 곱해서 다음 상태를 예측
        예: x_new = x_old + vx * dt
        */
        P_ = F_ * P_ * F_.transpose() + Q_;
        /* 
        공분산도 함께 예측, 시간이 지났으니 내 예측이 더 불확실해짐.
        F_ * P_ * F_.transpose()는 공분산을 다음 프레임으로 전이
        + Q_는 프로세스 노이즈를 추가 (내가 모르는 일이 있을 수도 있음.)
        값이 커지면, 분산은 제곱으로 커짐. 그래서 행렬은 앞뒤에 F_ 행렬을 곱해서 분산을 예측.
        */
    }
    
    void update(const geometry_msgs::Point& measurement)
    /* 업데이트 단계: 실제 측정값을 받아서 예측을 보정
    measurement는 LiDAR에서 측정한 실제 위치
    */
    {
        if (!initialized_)
        {
            init(measurement);
            /* 첫 측정값으로 초기화
            */
            return;
            /* 초기화하고 리턴 (업데이트는 다음에)
            */
        }
        
        // 관측 벡터
        Eigen::VectorXd z(3);
        /* 측정값을 담을 3차원 벡터 생성
        z는 관측(observation)을 의미
        */
        z << measurement.x, measurement.y, measurement.z;
        /* 벡터에 x, y, z 좌표를 채움
        << 연산자는 Eigen에서 벡터나 행렬에 값을 채우는 문법
        */
        
        // 칼만 게인 계산
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;  // 잔차 공분산
        /* 잔차(예측과 측정의 차이)의 공분산 계산
        H_ * P_ * H_.transpose()는 예측의 불확실성을 관측 공간으로 변환 (3x3 행렬)
        + R_는 측정 노이즈를 추가 (3x3 행렬)
        S가 크면 = 예측과 측정이 많이 다름 = 게인을 작게
        S가 작으면 = 예측과 측정이 비슷함 = 게인을 크게
        즉, P(예측 불확실성) + R(측정 불확실성)이라는거임.
        */
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();  // 칼만 게인
        /* 칼만 게인 핵심 공식
        게인은 "예측과 측정 중 어느 쪽을 더 신뢰할지"를 결정하는 비율
        K가 크면 = 측정값을 많이 반영
        K가 작으면 = 예측값을 많이 반영
        inverse()는 행렬의 역행렬 (나누기 같은 개념)
        예측 불확실성에 S(P+R)를 나눠서 계산하는거임(행렬에서 inverse는 나누는거랑 똑같음)
        */
        
        // 상태 업데이트
        Eigen::VectorXd y = z - H_ * state_;  // 잔차
        /* 잔차 계산: 실제 측정값 - 예측값
        y는 "예측이 얼마나 틀렸는지"를 나타냄
        y가 크면 = 예측이 많이 틀림
        y가 작으면 = 예측이 거의 맞음
        */
        state_ = state_ + K * y;
        /* 상태를 업데이트: 예측값 + 게인 * 잔차
        예측값에 잔차를 보정해서 더 정확한 상태를 얻음
        K * y는 "얼마나 보정할지"를 결정
        */
        
        // 공분산 업데이트
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        /* 6x6 단위 행렬 생성
        I는 Identity를 의미
        */
        P_ = (I - K * H_) * P_;
        /* 공분산을 업데이트
        측정값을 반영했으니 불확실성이 줄어듦
        (I - K * H_)는 불확실성을 줄이는 역할
        1에다가 K * H_를 빼서 P(불확실성)에다가 곱해서 불확실성을 줄임.
        */
    }
    
    geometry_msgs::Point getPosition() const
    /* 현재 추정된 위치를 반환하는 함수
    const는 이 함수가 객체를 수정하지 않는다는 의미
    */
    {
        geometry_msgs::Point pos;
        // ROS 메시지 타입의 Point 객체 생성
        pos.x = state_(0);
        pos.y = state_(1);
        pos.z = state_(2);
        return pos;
    }
    
    geometry_msgs::Point getVelocity() const
    /* 현재 추정된 속도를 반환하는 함수
    */
    {
        geometry_msgs::Point vel;
        /* 속도를 담을 Point 객체 생성
        (Point는 x, y, z를 담으니까 속도도 표현 가능)
        */
        vel.x = state_(3);
        vel.y = state_(4);
        vel.z = state_(5);
        return vel;
    }
    
    Eigen::MatrixXd getCovariance() const
    /* 현재 공분산 행렬을 반환하는 함수
    Mahalanobis 거리 계산에 필요
    */
    {
        return P_;
    }
    
    bool isInitialized() const
    /* 칼만 필터가 초기화되었는지 확인하는 함수
    */
    {
        return initialized_;
    }
};

// 헝가리안 알고리즘 (Munkres 알고리즘)
// 여러 트랙과 여러 detection을 최적으로 매칭하는 알고리즘
// 예: 트랙 3개, detection 3개가 있으면 3! = 6가지 조합이 있는데,
//     그 중에서 전체 비용이 가장 작은 조합을 찾음
class HungarianAlgorithm
{
public:
    // cost_matrix: n x m 행렬 (n: tracks, m: detections)
    // cost_matrix[i][j] = 트랙 i와 detection j를 매칭했을 때의 비용
    // 반환값: matched_pairs (track_idx -> detection_idx 매핑)
    // result[i] = j는 "트랙 i가 detection j와 매칭됨"을 의미
    // result[i] = -1은 "트랙 i가 매칭되지 않음"을 의미
    static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix)
    /* 헝가리안 알고리즘을 실행하는 정적 함수
    static은 객체를 만들지 않고도 호출할 수 있다는 의미
    */
    {
        if (cost_matrix.empty() || cost_matrix[0].empty())
        /* cost_matrix가 비어있으면
        */
        {
            return std::vector<int>(cost_matrix.size(), -1);
            /* 모든 트랙이 매칭되지 않았다는 의미로 -1을 채운 벡터 반환
            */
        }
        
        int n = cost_matrix.size();
        /* 트랙의 개수
        */
        int m = cost_matrix[0].size();
        /* detection의 개수
        cost_matrix[0]는 첫 번째 행 (첫 번째 트랙)
        그 행의 크기가 detection 개수
        */
        
        // 정사각 행렬로 만들기 (더 큰 쪽으로 패딩)
        int size = std::max(n, m);
        /* n과 m 중 더 큰 값
        헝가리안 알고리즘은 정사각 행렬에서 작동하니까
        */
        std::vector<std::vector<double>> matrix(size, std::vector<double>(size, 0.0));
        /* size x size 크기의 정사각 행렬 생성
        double 타입이고, 모든 값을 0.0으로 초기화
        */
        
        for (int i = 0; i < n; i++)
        /* 원본 cost_matrix의 각 행(트랙)에 대해
        */
        {
            for (int j = 0; j < m; j++)
            /* 원본 cost_matrix의 각 열(detection)에 대해
            */
            {
                matrix[i][j] = cost_matrix[i][j];
                /* 원본 비용 값을 새 행렬에 복사
                나머지 부분은 0으로 남음 (패딩)
                */
            }
        }
        
        // Munkres 알고리즘 실행
        std::vector<int> assignment = munkres(matrix);
        /* Munkres 알고리즘을 실행해서 최적 매칭을 찾음
        assignment[i] = j는 "행 i가 열 j와 매칭됨"을 의미
        */
        
        // 결과를 원래 크기로 조정
        std::vector<int> result(n, -1);
        /* 결과를 담을 벡터 생성 (트랙 개수만큼)
        모두 -1로 초기화 (매칭 안 됨)
        */
        for (int i = 0; i < n && i < size; i++)
        /* 각 트랙에 대해
        */
        {
            if (assignment[i] < m)
            /* 매칭된 detection이 실제 detection 범위 안에 있으면
            (패딩된 부분이 아닌 실제 detection)
            */
            {
                result[i] = assignment[i];
                /* 결과에 매칭 정보 저장
                */
            }
        }
        
        return result;
        /* 최적 매칭 결과 반환
        */
    }

private:
    static std::vector<int> munkres(std::vector<std::vector<double>>& matrix)
    /* Munkres 알고리즘의 간단한 버전
    완전한 Munkres 알고리즘은 복잡하니까 여기서는 간단한 그리디 버전 사용
    */
    {
        int n = matrix.size();
        /* 행렬의 크기 (정사각 행렬)
        */
        
        // Step 1: 각 행에서 최소값 빼기
        for (int i = 0; i < n; i++)
        /* 각 행에 대해
        */
        {
            double min_val = *std::min_element(matrix[i].begin(), matrix[i].end());//
            //begin(), end()는 반복자 반환(vector에 있는 멤버 함수)
            /* i번째 행에서 최소값을 찾음
            min_element는 최소값의 위치(반복자)를 반환(알고리즘에 있는 멤버 함수)
            *를 붙여서 실제 값을 가져옴
            */
            for (int j = 0; j < n; j++)
            /* 해당 행의 각 열에 대해
            */
            {
                matrix[i][j] -= min_val;
                /* 각 원소에서 최소값을 뺌
                이렇게 하면 각 행에 최소한 하나의 0이 생김
                */
            }
        }
        
        // Step 2: 각 열에서 최소값 빼기
        for (int j = 0; j < n; j++)
        /* 각 열에 대해
        */
        {
            double min_val = matrix[0][j];
            /* j번째 열의 첫 번째 원소를 최소값으로 초기화
            */
            for (int i = 1; i < n; i++)
            /* 나머지 행들을 확인
            */
            {
                min_val = std::min(min_val, matrix[i][j]);
                /* 더 작은 값으로 갱신
                */
            }
            for (int i = 0; i < n; i++)
            /* 해당 열의 각 행에 대해
            */
            {
                matrix[i][j] -= min_val;
                /* 각 원소에서 최소값을 뺌
                이렇게 하면 각 열에도 최소한 하나의 0이 생김
                */
            }
        }
        //이게 0이 있는 열들은 빼도 0빼는거니까 의미 없는건데,
        //0있는 것들 찾아서 그거 제외하는 if문 이런 거 연산이 더 손해임.
        //-빼는 연산은 가장 빠르고 싼 연산 중 하나여서 그냥 이거 ㄱㄱ
        
        // Step 3: 0을 찾아서 매칭 시도
        std::vector<int> assignment(n, -1);
        /* 매칭 결과를 담을 벡터
        assignment[i] = j는 "행 i가 열 j와 매칭됨"을 의미
        -1은 매칭 안 됨
        */
        std::vector<bool> row_covered(n, false);
        /* 각 행이 이미 매칭되었는지 표시
        true면 이미 매칭됨
        */
        std::vector<bool> col_covered(n, false);
        /* 각 열이 이미 매칭되었는지 표시
        true면 이미 매칭됨
        */
        
        // 간단한 그리디 매칭 (0인 셀 찾기)
        for (int i = 0; i < n; i++)
        /* 각 행에 대해
        */
        {
            for (int j = 0; j < n; j++)
            /* 각 열에 대해
            */
            {
                if (matrix[i][j] == 0.0 && !row_covered[i] && !col_covered[j])
                /* 해당 셀이 0이고, 행도 열도 아직 매칭되지 않았으면
                */
                {
                    assignment[i] = j;
                    /* 행 i와 열 j를 매칭
                    */
                    row_covered[i] = true;
                    /* 행 i를 매칭됨으로 표시
                    */
                    col_covered[j] = true;
                    /* 열 j를 매칭됨으로 표시
                    */
                    break;
                    /* 이 행은 매칭 완료했으니 다음 행으로
                    */
                }
            }
        }
        
        return assignment;
        /* 매칭 결과 반환
        */
    }
};


class GigachaLidarTracking
/* 메인 트래킹 클래스
lidar.cpp에서 생성한 바운딩 박스들을 받아서 여러 프레임에 걸쳐 추적
*/
{
private:
    ros::NodeHandle nh_;
    /* ROS 노드 핸들
    이 프로그램이 ROS 시스템과 소통하기 위한 핵심 매니저
    Publisher, Subscriber, 파라미터 로드 등을 담당
    */
    ros::Subscriber detection_sub_;
    /* 수신자
    lidar.cpp에서 보내는 바운딩 박스 데이터를 받음
    토픽 이름: /gigacha/lidar/bounding_boxes
    */
    ros::Publisher tracking_pub_;
    /* 송신자
    트래킹된 결과를 다른 노드로 보냄
    토픽 이름: /gigacha/lidar/tracked_objects
    */
    
    // 트래킹 파라미터
    float max_mahalanobis_distance_;  // Mahalanobis 거리 임계값
    /* Mahalanobis 거리의 최대 허용값
    이 값보다 크면 매칭하지 않음
    */
    float max_disappeared_frames_;
    /* 객체가 몇 프레임 동안 사라져도 트랙을 유지할지
    예: 5면 5프레임 동안 안 보여도 트랙 유지, 그 이후엔 삭제
    */
    float gating_threshold_;           // 게이팅 임계값 (chi-square, 3 DOF)
    /* 게이팅 임계값
    불가능한 매칭을 미리 제거하는 용도
    chi-square 분포에서 3 자유도, 99% 신뢰도 기준으로 약 9.0
    이 값보다 Mahalanobis 거리가 크면 매칭하지 않음
    */
    //그냥 오차 정도면 매칭하는거고, 크면 매칭 안하는거임.
    //정규분포를 따르는 변수들의 제곱 합(마할라노비스 거리의 제곱)은 카이제곱 분포를 따름.
    
    // 트랙 관리
    struct Track
    /* 하나의 추적 대상(객체)에 대한 정보를 담는 구조체
    예: 차량 1대, 보행자 1명 등
    */
    {
        int track_id;
        /* 이 트랙의 고유 ID
        예: 1, 2, 3, ...
        같은 객체는 항상 같은 ID를 가짐
        */
        KalmanFilter3D kf;                    // 칼만 필터
        /* 이 트랙의 칼만 필터 객체
        위치와 속도를 추정하고 예측하는 역할
        */
        vision_msgs::Detection3D detection;   // 최신 detection
        /* 가장 최근에 받은 detection 정보
        바운딩 박스, 크기 등이 들어있음
        */
        int disappeared_count;
        /* 연속으로 몇 프레임 동안 사라졌는지
        0이면 현재 프레임에서 보임
        1이면 1프레임 전에 마지막으로 봄
        */
        ros::Time last_seen;
        /* 마지막으로 이 객체를 본 시간
        시간 차이를 계산하는 데 사용
        */
        int age;                              // 트랙 나이 (프레임 수)
        /* 이 트랙이 생성된 후 몇 프레임이 지났는지
        나이가 많을수록 신뢰도가 높음
        */
    };
    
    std::vector<Track> tracks_;
    /* 현재 활성화된 모든 트랙들의 목록
    예: 차량 3대, 보행자 2명이 추적 중이면 5개의 Track이 들어있음
    */
    int next_track_id_;
    /* 다음에 할당할 트랙 ID
    새로운 객체가 발견되면 이 값을 사용하고 1 증가
    */
    ros::Time last_update_time_;
    /* 마지막으로 업데이트한 시간
    시간 차이(dt)를 계산하는 데 사용
    */
    
    // detection의 중심점 추출
    geometry_msgs::Point getCenter(const vision_msgs::Detection3D& det)
    /* Detection3D 메시지에서 중심점을 추출하는 함수
    */
    {
        return det.bbox.center.position;
        //detection3D(장애물 정보) 안에 .bbox(Bounding Box)에 이제 박스 크기 정보가 있고,
        //.center(중심점)에 박스 중심점 위치 정보(position)가 있음.
    }
    
    // Mahalanobis 거리 계산
    // predicted: 칼만 필터 예측 위치
    // measurement: 실제 측정 위치
    // covariance: 칼만 필터 공분산 행렬
    double calculateMahalanobisDistance(
        const geometry_msgs::Point& predicted,
        const geometry_msgs::Point& measurement,
        const Eigen::MatrixXd& covariance)
    /* Mahalanobis 거리를 계산하는 함수
    유클리드 거리와 달리 불확실성을 고려한 통계적 거리
    공분산이 큰 방향으로는 거리가 작게 계산됨 (불확실성이 크니까)
    공분산이 작은 방향으로는 거리가 크게 계산됨 (불확실성이 작으니까)
    */
    {
        Eigen::Vector3d pred(predicted.x, predicted.y, predicted.z);
        /* 예측 위치를 Eigen 벡터로 변환
        Vector3d는 3차원 double 벡터
        */
        Eigen::Vector3d meas(measurement.x, measurement.y, measurement.z);
        /* 측정 위치를 Eigen 벡터로 변환
        */
        Eigen::Vector3d diff = meas - pred;
        /* 차이 벡터: 측정값 - 예측값
        예측이 맞으면 diff가 작음
        */
        
        /*
        공분산 : 두 변수가 얼마나 함께 변하는지를 나타내는 말임.
        자율주행 자동차에서는 공분산을 많이 사용하지. 내 차는 달리고 있으니까
        그냥 분산 : 내가 혼자서 얼마나 변하는지를 나타내는 말임.
        */

        // 관측 공분산 (H * P * H^T)
        Eigen::Matrix3d S = covariance.block<3, 3>(0, 0);
        /* 공분산 행렬에서 위치 부분만 추출
        공분산 행렬은 6x6인데 (위치 3개 + 속도 3개)
        위치 부분만 3x3으로 추출
        block<3, 3>(0, 0)는 (0, 0) 위치에서 3x3 크기만큼 추출
        */
        
        // Mahalanobis 거리 = sqrt((x - mu)^T * S^(-1) * (x - mu))
        Eigen::Matrix3d S_inv = S.inverse();
        /* 공분산 행렬의 역행렬 계산
        S^(-1)는 공분산의 역행렬
        */
        double mahal_dist = std::sqrt(diff.transpose() * S_inv * diff);
        /* Mahalanobis 거리 계산
        1X3*3X3*3X1 이어서 스칼라 값이 나옴.
        diff.transpose()는 diff 벡터를 전치 (행 벡터로 변환)
        diff.transpose() * S_inv * diff는 스칼라 값 (거리의 제곱) (1X1)
        sqrt를 해서 루트 씌워줘서 실제 거리로 변환
        */
        
        return mahal_dist;
        /* 계산된 Mahalanobis 거리 반환
        */
    }
    
    // Cost Matrix 생성 (Mahalanobis 거리 기반)
    std::vector<std::vector<double>> buildCostMatrix(
        const vision_msgs::Detection3DArray& detections)
    /* Cost Matrix를 생성하는 함수
    Cost Matrix는 각 트랙- detection 쌍의 비용을 담은 행렬
    비용이 작을수록 매칭하기 좋음
    */
    {
        std::vector<std::vector<double>> cost_matrix;
        /* 비용 행렬을 담을 2차원 벡터
        cost_matrix[i][j] = 트랙 i와 detection j를 매칭했을 때의 비용
        */
        
        if (tracks_.empty() || detections.detections.empty())
        /* 트랙이 없거나 detection이 없으면
        */
        {
            return cost_matrix;
            /* 빈 행렬 반환
            */
        }
        
        cost_matrix.resize(tracks_.size());
        /* 행의 개수를 트랙 개수로 설정
        각 행은 하나의 트랙을 의미
        */
        
        for (size_t i = 0; i < tracks_.size(); i++)
        /* 각 트랙에 대해
        */
        {
            cost_matrix[i].resize(detections.detections.size());
            /* 해당 행의 열 개수를 detection 개수로 설정
            각 열은 하나의 detection을 의미
            */
            
            // 칼만 필터 예측 위치
            geometry_msgs::Point predicted_pos = tracks_[i].kf.getPosition();
            //kf는 track 구조체에 있는것것
            /* i번째 트랙의 칼만 필터에서 예측 위치를 가져옴
            이건 "다음 프레임에 이 객체가 어디 있을지" 예측한 위치
            */
            Eigen::MatrixXd covariance = tracks_[i].kf.getCovariance();
            /* i번째 트랙의 칼만 필터에서 공분산 행렬을 가져옴
            불확실성을 나타냄
            */
            
            for (size_t j = 0; j < detections.detections.size(); j++)
            /* 각 detection에 대해
            */
            {
                geometry_msgs::Point det_center = getCenter(detections.detections[j]);
                /* j번째 detection의 중심점을 가져옴
                */
                
                // Mahalanobis 거리 계산
                double mahal_dist = calculateMahalanobisDistance(
                    predicted_pos, det_center, covariance);
                    //kf 예측 포지션, detection 센터, kf 공분산산
                /* 트랙 i의 예측 위치와 detection j의 실제 위치 사이의
                Mahalanobis 거리를 계산
                */
                
                // 게이팅: 임계값을 넘으면 매우 큰 cost 할당
                if (mahal_dist > gating_threshold_)
                /* Mahalanobis 거리가 게이팅 임계값보다 크면
                (불가능한 매칭)
                */
                {
                    cost_matrix[i][j] = 1e6;  // 매우 큰 값
                    /* 비용을 매우 크게 설정
                    헝가리안 알고리즘이 이 매칭을 선택하지 않도록
                    1e6은 1,000,000 (백만)
                    */
                }
                else
                /* Mahalanobis 거리가 게이팅 임계값 이하면
                (가능한 매칭)
                */
                {
                    // 정규화된 Mahalanobis 거리를 cost로 사용
                    cost_matrix[i][j] = mahal_dist;
                    /* Mahalanobis 거리를 그대로 비용으로 사용
                    거리가 가까울수록 비용이 작음 = 매칭하기 좋음
                    */
                }
            }
        }
        
        return cost_matrix;
        /* 완성된 비용 행렬 반환
        */
    }
    
    // 헝가리안 알고리즘을 사용한 최적 매칭
    void associateDetectionsToTracks(
        const vision_msgs::Detection3DArray& detections,
        std::vector<int>& matched_tracks,
        std::vector<int>& unmatched_detections)
    /* 트랙과 detection을 매칭하는 함수
    matched_tracks: 각 트랙이 어떤 detection과 매칭되었는지
                   matched_tracks[i] = j는 "트랙 i가 detection j와 매칭됨"
                   matched_tracks[i] = -1은 "트랙 i가 매칭 안 됨"
    unmatched_detections: 매칭되지 않은 detection들의 인덱스
    */
    {
        matched_tracks.clear();
        /* 결과 벡터 초기화
        */
        unmatched_detections.clear();
        /* 결과 벡터 초기화
        */
        
        if (tracks_.empty())
        /* 트랙이 없으면
        */
        {
            for (int i = 0; i < (int)detections.detections.size(); i++)
            /* 모든 detection에 대해
            */
            {
                unmatched_detections.push_back(i);
                /* 모두 매칭되지 않은 것으로 표시
                (새로운 객체로 간주)
                */
            }
            return;
            /* 함수 종료
            */
        }
        
        if (detections.detections.empty())
        /* detection이 없으면
        */
        {
            matched_tracks.resize(tracks_.size(), -1);
            /* 모든 트랙이 매칭되지 않았다고 표시
            */
            return;
            /* 함수 종료
            */
        }
        
        // Cost Matrix 생성
        std::vector<std::vector<double>> cost_matrix = buildCostMatrix(detections);
        /* 각 트랙-detection 쌍의 비용을 담은 행렬 생성
        */
        
        // 헝가리안 알고리즘으로 최적 매칭
        matched_tracks = HungarianAlgorithm::solve(cost_matrix);
        // solve는 벡터 형태로 반환됨. 
        /* 헝가리안 알고리즘을 실행해서 최적 매칭을 찾음
        전체 비용이 최소가 되는 매칭을 찾음
        */
        
        // 매칭되지 않은 detection 찾기
        std::vector<bool> det_matched(detections.detections.size(), false);
        /* 각 detection이 매칭되었는지 표시하는 벡터
        false = 매칭 안 됨
        true = 매칭 됨
        */
        for (size_t i = 0; i < matched_tracks.size(); i++)
        /* 각 트랙에 대해
        */
        {
            if (matched_tracks[i] >= 0 && matched_tracks[i] < (int)detections.detections.size())
            //뒤에 조건은 detections.detections.size() 전체 사이즈보다 matched_tracks[i]가 안에 있냐
            //존재하는 번호냐? 물어보는 조건임.
            /* 트랙 i가 유효한 detection과 매칭되었으면
            (matched_tracks[i] >= 0은 매칭됨을 의미)
            */
            {
                det_matched[matched_tracks[i]] = true;
                /* 해당 detection을 매칭됨으로 표시
                matched_tracks[i]는 detection의 인덱스
                */
            }
        }
        
        //앞 detections는 코드에서 선언한 변수 이름(택배 상자). 뒤에 detections는 실제 리스트 이름(내용물)
        for (size_t j = 0; j < detections.detections.size(); j++)
        /* 각 detection에 대해
        */
        {
            if (!det_matched[j])
            /* 매칭되지 않은 detection이면
            */
            {
                unmatched_detections.push_back(j);
                /* unmatched_detections에 추가
                (새로운 객체로 간주)
                */
            }
        }
    }
    
    // 트랙 업데이트 (칼만 필터 사용)
    void updateTrack(Track& track, const vision_msgs::Detection3D& detection, double dt)
    /* 트랙을 업데이트하는 함수
    새로운 detection을 받아서 칼만 필터를 업데이트하고 트랙 정보 갱신
    dt는 이전 업데이트와 현재 사이의 시간 차이
    */
    {
        geometry_msgs::Point measurement = getCenter(detection);
        /* detection의 중심점을 측정값으로 사용
        */
        
        // 칼만 필터 예측
        track.kf.predict(dt);
        /* 칼만 필터로 다음 위치를 예측
        dt 시간이 지났으니 객체가 어디로 이동했을지 예측
        */
        
        // 칼만 필터 업데이트
        track.kf.update(measurement);
        /* 실제 측정값으로 칼만 필터를 업데이트
        예측과 측정을 합쳐서 더 정확한 상태를 얻음
        */
        
        // detection 정보 업데이트
        track.detection = detection;
        /* 트랙의 detection 정보를 최신 것으로 갱신
        */
        track.disappeared_count = 0;
        /* 사라진 프레임 수를 0으로 리셋
        (현재 프레임에서 보였으니까)
        */
        track.last_seen = ros::Time::now();
        /* 마지막으로 본 시간을 현재 시간으로 업데이트
        */
        track.age++;
        /* 트랙 나이를 1 증가
        */
    }
    
    // 사라진 트랙 처리 (칼만 필터로 예측만 수행)
    void updateDisappearedTracks(double dt)
    /* 사라진 트랙들을 처리하는 함수
    detection이 없어도 칼만 필터로 예측은 계속 수행
    나중에 다시 나타날 수 있으니까
    */
    {
        for (auto& track : tracks_)
        /* 모든 트랙에 대해
        */
        {
            if (track.disappeared_count > 0)
            /* 사라진 트랙이면
            (disappeared_count > 0은 이전 프레임에서도 안 보였다는 의미)
            */
            {
                // 칼만 필터로 예측만 수행 (관측 없음)
                track.kf.predict(dt);
                /* 칼만 필터로 예측만 수행
                측정값이 없으니까 update()는 호출 안 함
                예측만 해서 "다음에 어디 있을지" 추정
                */
            }
        }
    }
    
    // 오래된 트랙 제거
    void removeOldTracks()
    /* 너무 오래 사라진 트랙들을 제거하는 함수
    max_disappeared_frames_보다 오래 사라진 트랙은 삭제
    */
    {
        tracks_.erase(
            /* erase는 벡터에서 요소를 제거하는 함수
            */
            std::remove_if(tracks_.begin(), tracks_.end(),
                /* remove_if는 조건에 맞는 요소들을 뒤로 이동시키는 함수
                실제로는 삭제하지 않고 뒤로 보냄
                begin()과 end()는 벡터의 시작과 끝
                */
                [this](const Track& track) {
                    /* 람다 함수: 각 트랙에 대해 조건을 확인
                    [this]는 이 클래스의 멤버 변수에 접근하기 위한 캡처
                    max_disappeared_frams 접근하기 위한 거거
                    */
                    return track.disappeared_count > max_disappeared_frames_;
                    /* disappeared_count가 임계값보다 크면 true 반환
                    (제거 대상)
                    */
                }),
                //remove_if는 iterator 반환 여기서부터 쓰레기값이에요를 알려줌.
            tracks_.end());
        /* remove_if가 반환한 위치부터 end()까지를 erase로 제거
        이렇게 하면 실제로 삭제됨
        */
    }
    
    // 새로운 트랙 생성
    void createNewTracks(const vision_msgs::Detection3DArray& detections,
                        const std::vector<int>& unmatched_detections)
    /* 매칭되지 않은 detection들을 새로운 트랙으로 생성하는 함수
    새로운 객체가 나타났다는 의미
    */
    {
        for (int det_idx : unmatched_detections)
        /* 매칭되지 않은 각 detection에 대해
        det_idx는 detection의 인덱스
        */
        {
            Track new_track;
            /* 새로운 트랙 객체 생성
            */
            new_track.track_id = next_track_id_++;
            /* 고유한 트랙 ID 할당
            next_track_id_를 사용하고 1 증가
            예: 첫 번째 객체면 1, 두 번째면 2, ...
            */
            new_track.detection = detections.detections[det_idx];
            /* detection 정보를 트랙에 저장
            */
            new_track.disappeared_count = 0;
            /* 사라진 프레임 수를 0으로 설정
            (현재 프레임에서 보였으니까)
            */
            new_track.last_seen = ros::Time::now();
            /* 마지막으로 본 시간을 현재 시간으로 설정
            */
            new_track.age = 1;
            /* 트랙 나이를 1로 설정
            (방금 생성됐으니까)
            */
            
            // 칼만 필터 초기화
            geometry_msgs::Point init_pos = getCenter(detections.detections[det_idx]);
            /* detection의 중심점을 가져옴
            */
            new_track.kf.init(init_pos);
            /* 칼만 필터를 초기 위치로 초기화
            첫 측정값으로 칼만 필터를 시작
            */
            
            tracks_.push_back(new_track);
            /* 새로운 트랙을 트랙 목록에 추가
            */
        }
    }
    
public:
    GigachaLidarTracking() : nh_("~"), next_track_id_(1)
    /* 생성자: 트래킹 노드를 초기화
    nh_("~")는 private 노드 핸들을 생성 (파라미터 충돌 방지)
    next_track_id_(1)는 첫 번째 트랙 ID를 1로 설정
    1번부터 시작하겠다다
    */
    {
        ROS_INFO("GIGACHA LiDAR Tracking Node (Kalman + Hungarian) Starting...");
        /* ROS 로그 출력
        노드가 시작되었다는 메시지
        */
        
        // 파라미터 로드
        nh_.param<float>("max_mahalanobis_distance", max_mahalanobis_distance_, 3.0f);
        /* ROS 파라미터 서버에서 max_mahalanobis_distance 값을 읽어옴
        없으면 기본값 3.0 사용
        */
        nh_.param<float>("max_disappeared_frames", max_disappeared_frames_, 5.0f);
        /* max_disappeared_frames 파라미터 읽기
        없으면 기본값 5 사용
        */
        nh_.param<float>("gating_threshold", gating_threshold_, 9.0f);  // chi-square 3 DOF, 99% 신뢰도
        /* gating_threshold 파라미터 읽기
        없으면 기본값 9.0 사용
        chi-square 분포에서 3 자유도, 99% 신뢰도 기준
        */
        
        // Publisher/Subscriber 설정
        detection_sub_ = nh_.subscribe("/gigacha/lidar/bounding_boxes", 1,
                                       &GigachaLidarTracking::detectionCallback, this);
        /* Subscriber 생성
        "/gigacha/lidar/bounding_boxes" 토픽을 구독
        데이터가 오면 detectionCallback 함수가 자동으로 호출됨
        1은 큐 사이즈 (최신 데이터 1개만 유지)
        this는 이 클래스의 멤버 함수라는 의미
        */
        tracking_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/gigacha/lidar/tracked_objects", 1);
        /* Publisher 생성
        "/gigacha/lidar/tracked_objects" 토픽으로 트래킹 결과를 발행
        vision_msgs::Detection3DArray 타입의 메시지를 보냄
        1은 큐 사이즈
        */
        
        last_update_time_ = ros::Time::now();
        /* 마지막 업데이트 시간을 현재 시간으로 설정
        */
        
        ROS_INFO("GIGACHA LiDAR Tracking Node Initialized");
        /* 초기화 완료 메시지
        */
        ROS_INFO("Parameters:");
        /* 파라미터 정보 출력 시작
        */
        ROS_INFO("  Max Mahalanobis Distance: %.2f", max_mahalanobis_distance_);
        /* Max Mahalanobis Distance 파라미터 값 출력
        */
        ROS_INFO("  Max Disappeared Frames: %.0f", max_disappeared_frames_);
        /* Max Disappeared Frames 파라미터 값 출력
        */
        ROS_INFO("  Gating Threshold: %.2f", gating_threshold_);
        /* Gating Threshold 파라미터 값 출력
        */
    }
    
    ~GigachaLidarTracking()
    /* 소멸자: 노드가 종료될 때 호출
    */
    {
        ROS_INFO("GIGACHA LiDAR Tracking Node Terminated");
        /* 종료 메시지 출력
        */
    }
    
    void detectionCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    /* 콜백 함수: 바운딩 박스 데이터가 올 때마다 자동으로 호출됨
    ConstPtr는 상수 스마트 포인터 (데이터를 읽기만 함)
    */
    {
        ros::Time current_time = ros::Time::now();
        /* 현재 시간을 가져옴
        */
        double dt = (current_time - last_update_time_).toSec();
        /* 이전 업데이트와 현재 사이의 시간 차이 계산 (초 단위)
        toSec()는 ros::Time을 초 단위 double로 변환
        */
        if (dt < 0.001 || dt > 1.0) dt = 0.1;  // 기본값 설정
        /* 시간 차이가 너무 작거나 크면 기본값 0.1초 사용
        dt < 0.001: 거의 0이면 (같은 프레임일 수 있음)
        dt > 1.0: 1초보다 크면 (데이터가 너무 늦게 옴)
        */
        last_update_time_ = current_time;
        /* 마지막 업데이트 시간을 현재 시간으로 갱신
        */
        
        if (msg->detections.empty())
        /* detection이 없으면
        (주변에 객체가 없음)
        */
        {
            // detection이 없으면 칼만 필터로 예측만 수행
            updateDisappearedTracks(dt);
            /* 모든 트랙에 대해 칼만 필터로 예측만 수행
            측정값이 없으니까 예측만 함
            */
            removeOldTracks();
            /* 오래 사라진 트랙들을 제거
            */
            
            vision_msgs::Detection3DArray tracked_array;
            /* 빈 트래킹 결과 배열 생성
            */
            tracked_array.header = msg->header;
            /* 헤더 정보 복사 (타임스탬프, 좌표계 등)
            */
            tracking_pub_.publish(tracked_array);
            /* 빈 결과를 발행
            (객체가 없으니까)
            */
            return;
            /* 함수 종료
            */
        }
        
        // 트랙과 detection 매칭 (헝가리안 알고리즘)
        std::vector<int> matched_tracks;
        /* 매칭된 트랙들의 결과를 담을 벡터
        matched_tracks[i] = j는 "트랙 i가 detection j와 매칭됨"
        */
        std::vector<int> unmatched_detections;
        /* 매칭되지 않은 detection들의 인덱스
        새로운 객체로 간주됨
        */
        associateDetectionsToTracks(*msg, matched_tracks, unmatched_detections);
        /* 헝가리안 알고리즘으로 최적 매칭 수행
        *msg는 포인터를 역참조해서 실제 메시지 객체를 가져옴
        */
        
        // 매칭된 트랙 업데이트 (칼만 필터)
        for (size_t i = 0; i < tracks_.size(); i++)
        /* 각 트랙에 대해
        */
        {
            if (matched_tracks[i] >= 0 && matched_tracks[i] < (int)msg->detections.size())
            /* 트랙 i가 유효한 detection과 매칭되었으면
            matched_tracks[i] >= 0은 매칭됨을 의미
            matched_tracks[i] < size는 인덱스가 유효함을 의미
            */
            {
                updateTrack(tracks_[i], msg->detections[matched_tracks[i]], dt);
                /* 트랙을 업데이트
                matched_tracks[i]는 매칭된 detection의 인덱스
                */
            }
            else
            /* 트랙 i가 매칭되지 않았으면
            (이번 프레임에서 안 보임)
            */
            {
                tracks_[i].disappeared_count++;
                /* 사라진 프레임 수를 1 증가
                */
                // 칼만 필터로 예측만 수행
                tracks_[i].kf.predict(dt);
                /* 칼만 필터로 예측만 수행
                측정값이 없으니까 update()는 호출 안 함
                */
            }
        }
        
        // 새로운 트랙 생성
        createNewTracks(*msg, unmatched_detections);
        /* 매칭되지 않은 detection들을 새로운 트랙으로 생성
        새로운 객체가 나타났다는 의미
        */
        
        // 오래된 트랙 제거
        removeOldTracks();
        /* 너무 오래 사라진 트랙들을 제거
        max_disappeared_frames_보다 오래 사라진 트랙 삭제
        */
        
        // 트래킹된 결과 생성
        vision_msgs::Detection3DArray tracked_array;
        /* 트래킹 결과를 담을 배열 생성
        */
        tracked_array.header = msg->header;
        /* 헤더 정보 복사 (타임스탬프, 좌표계 등)
        */
        
        for (const auto& track : tracks_)
        /* 각 트랙에 대해
        const auto&는 타입을 자동으로 추론하고 상수 참조로 가져옴
        */
        {
            if (track.disappeared_count == 0)  // 현재 프레임에서 보인 트랙만
            /* 현재 프레임에서 보인 트랙만
            (disappeared_count == 0은 이번 프레임에서 보였다는 의미)
            */
            {
                vision_msgs::Detection3D tracked_det = track.detection;
                /* 트랙의 detection 정보를 복사
                */
                
                // 칼만 필터 예측 위치로 업데이트 (더 정확한 위치)
                geometry_msgs::Point kf_pos = track.kf.getPosition();
                /* 칼만 필터에서 추정한 위치를 가져옴
                이게 측정값보다 더 정확함 (여러 프레임의 정보를 합쳤으니까)
                */
                tracked_det.bbox.center.position = kf_pos;
                /* 바운딩 박스의 중심 위치를 칼만 필터 위치로 업데이트
                */
                
                // 트랙 ID를 results에 추가
                if (tracked_det.results.empty())
                /* results가 비어있으면
                results는 라벨링 같은거임. 뭐 이건 차다 이건 트럭이다 나무다.
                우리는 lidar를 clustering했으니까 그냥 점군이잖아 그래서 항상 label 비어있음.
                */
                {
                    vision_msgs::ObjectHypothesisWithPose hypothesis;
                    /* 새로운 hypothesis 객체 생성
                    hypothesis는 "이 객체가 뭔지"에 대한 추정
                    */
                    hypothesis.id = track.track_id;
                    /* 트랙 ID를 hypothesis의 id에 저장
                    이렇게 하면 다른 노드에서 트랙 ID를 알 수 있음
                    */
                    hypothesis.score = 1.0;
                    /* 신뢰도를 1.0 (100%)으로 설정
                    LiDAR로 직접 측정했으니까 100% 신뢰
                    */
                    tracked_det.results.push_back(hypothesis);
                    /* hypothesis를 results에 추가
                    */
                }
                else
                /* 
                results가 이미 있으면
                얘는 이제 딥러닝하면, 그냥 이거 몇번이다만 부착해주는거지.
                */
                {
                    tracked_det.results[0].id = track.track_id;
                    /* 첫 번째 hypothesis의 id를 트랙 ID로 업데이트
                    */
                }
                
                tracked_array.detections.push_back(tracked_det);
                /* 트래킹된 detection을 결과 배열에 추가
                */
            }
        }
        
        // 결과 publish
        tracking_pub_.publish(tracked_array);
        /* 트래킹 결과를 발행
        다른 노드들이 이 데이터를 받아서 사용할 수 있음
        */
        
        ROS_DEBUG("Active tracks: %zu, Detections: %zu, Matched: %zu",
                  tracks_.size(),
                  msg->detections.size(),
                  std::count_if(matched_tracks.begin(), matched_tracks.end(),
                               [](int x) { return x >= 0; }));
        /* 디버그 로그 출력
        활성 트랙 수, detection 수, 매칭된 수를 출력
        ROS_DEBUG는 디버그 모드에서만 출력됨
        count_if는 조건에 맞는 요소의 개수를 세는 함수
        [](int x) { return x >= 0; }는 람다 함수 (매칭된 것만 셈)
        */
    }
};

int main(int argc, char **argv)
/* 메인 함수: 프로그램의 시작점
argc는 인자 개수, argv는 인자 값들 (문자열 배열)
*/
{
    ros::init(argc, argv, "gigacha_lidar_tracking");
    /* ROS 노드 초기화
    "gigacha_lidar_tracking"은 노드 이름
    ROS 시스템에 이 노드를 등록
    */
    GigachaLidarTracking node;
    /* 트래킹 노드 객체 생성
    생성자가 호출되면서 초기화됨
    */
    ros::spin();
    /* ROS 이벤트 루프 시작
    콜백 함수가 호출되기를 기다리는 무한 루프
    데이터가 오면 자동으로 detectionCallback이 호출됨
    */
    return 0;
    /* 프로그램 종료
    (실제로는 ros::spin()이 무한 루프라서 여기까지 오지 않음)
    */
}
