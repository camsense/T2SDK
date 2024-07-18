
/**
 * @author yx.liu
 * @date 2024/2/20 11:28
 * @version 1.0
 */
#ifndef CAMSENSE_SDK_T2OUTLIERREMOVAL_H
#define CAMSENSE_SDK_T2OUTLIERREMOVAL_H

#include <Eigen/Dense>
#include "HcData.h"
#define BUFFER_SIZE 31
using namespace std;
using namespace Eigen;

static constexpr struct SmoothParam{
    int MaxDiff = 80;
    int MaxBiasThreshold = 50;
    int RepeatTimes = 1;
    int KernelSize = 9;
    int Poly = 1;
    int MaxFilterDistance = 1000;
} smoothParam;

#define PI 3.141592653589793
class Point2fData
{
public:
    Point2fData() = default;
    Point2fData(const Point2fData &sOthers) {
        if (this == &sOthers)
            return;
        x = sOthers.x;   //angle
        fRawAngle = sOthers.fRawAngle;
        fOffSetAngle = sOthers.fOffSetAngle;
        y = sOthers.y;  //distance
        fRawDistance = sOthers.fRawDistance;
        alpha = sOthers.alpha; //gray
        type = sOthers.type; // 1: 三角测距; 0: TOF测距
        bIsInVaid = sOthers.bIsInVaid;  //1无效点  0 有效点
        bIsFilter = sOthers.bIsFilter;
        fSpeed = sOthers.fSpeed;
        fx = sOthers.fx;
        fy = sOthers.fy;
        nOutlierFlag = sOthers.nOutlierFlag;
        bIsSmooth = sOthers.bIsSmooth;
        fDist = sOthers.fDist;
        min_score = sOthers.min_score;
        bIs3M = sOthers.bIs3M;
    }
    Point2fData & operator = (const Point2fData &sOthers) {
        if (this != &sOthers) {
            x = sOthers.x;   //angle
            fRawAngle = sOthers.fRawAngle;
            fOffSetAngle = sOthers.fOffSetAngle;
            y = sOthers.y;  //distance
            fRawDistance = sOthers.fRawDistance;
            alpha = sOthers.alpha; //gray
            type = sOthers.type; // 1: 三角测距; 0: TOF测距
            bIsInVaid = sOthers.bIsInVaid;  //1无效点  0 有效点
            bIsFilter = sOthers.bIsFilter;
            fSpeed = sOthers.fSpeed;
            fx = sOthers.fx;
            fy = sOthers.fy;
            nOutlierFlag = sOthers.nOutlierFlag;
            bIsSmooth = sOthers.bIsSmooth;
            fDist = sOthers.fDist;
            min_score = sOthers.min_score;
            bIs3M = sOthers.bIs3M;
        }
        return *this;
    }
    float x = { 0.0 };   //angle
    float fRawAngle = { 0.0 };
    float fOffSetAngle = { 0.0 };
    float y = { 0.0 };  //distance
    float fRawDistance = { 0.0 };
    float alpha = { 0.0 }; //gray
    int type = { 0 }; // 1: 三角测距; 0: TOF测距
    bool bIsInVaid = { true };  //1无效点  0 有效点
    bool bIsFilter = { false };  //杂点过滤  1过滤  0不过滤
    float fSpeed = { 0.0 };//
    float fx = { 0.0 }; //直角坐标x
    float fy = { 0.0 };//直角坐标y
    uint8_t  nOutlierFlag = { 0 };
    bool bIsSmooth = { false };
    float fDist = { 0.0 };
    float min_score = { 1.0 };
    bool bIs3M = { false };
};


class T2OutlierRemoval {
public:
    T2OutlierRemoval();
    ~T2OutlierRemoval();
    void preprocess_pointcloud(std::vector<Point2fData>& pointcloud_data);
    void smoth_pointCloud(std::vector<Point2fData> & sPointCloud,const int nRepeatTimes,const float fMaxDiff, const float fMaxBiasThr,const uint8_t nKernelSize = 4,const double dbDistance = 1000);
    void weightSmooth_pointCloud(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes,const float fMaxGap,const float fMaxDiff,const double dbDistance = 1000);
    void guassSmooth_pointCloud(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes,const float fMaxGap,const float fMaxDiff,const double dbDistance = 1000);
    bool setWeightSmoothParameter(const int nKernelSize, const int nFitPoly);
    bool setGuassSmoothParameter(const int nKernelSize);
private:
    enum OUTLIERFLAG
    {
        NORMAL = 0,
        LEVEL1,
        LEVEL2,
        LEVEL3
    };
    bool check_line(const std::vector<Point2fData> &supports);
    void update_arr(Point2fData &newdata);
    void calculate_dist_eu(uint8_t ids, uint8_t ide, float dist_ratio, float&mean_dist, uint8_t& count,std::vector<Point2fData>& supports);
    uint8_t outlier_removal(float& score);
    uint8_t calc_cossim(const Point2fData &point_data,const std::vector<Point2fData> &supports, float& min_score, int& ft_cnt);
    int smooth_4points_once(vector<Point2fData> &data, const float fMaxDiff, const float fMaxBiasThr, const uint8_t nKernelSize = 4,const double dbDistance = 1000);
    bool getGuassKernel(const int n);
    void doSmoothProcessing(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes, const float fMaxGap,const float fMaxDiff, const VectorXd &sWeight,const double dbDistance = 1000);
    std::vector<Point2fData> process_arr_;
    VectorXd m_sWeigthSeed,m_sGuassKernel;
};

void smoothPointCloud(LstPointCloud &pointCloud, int iSmoothMode, T2OutlierRemoval &t2removal);


#endif //CAMSENSE_SDK_T2OUTLIERREMOVAL_H
