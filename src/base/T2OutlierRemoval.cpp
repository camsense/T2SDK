
/**
 * @author yx.liu
 * @date 2024/2/20 11:28
 * @version 1.0
 */
#include "T2OutlierRemoval.h"
#include <iostream>

T2OutlierRemoval::T2OutlierRemoval()
{

};
T2OutlierRemoval::~T2OutlierRemoval()
{

};

void T2OutlierRemoval::preprocess_pointcloud(std::vector<Point2fData>& pointcloud_data)
{
    if (pointcloud_data.size() < BUFFER_SIZE)
        return;
    process_arr_.clear();
    Point2fData new_data;
    uint8_t ret_val = 0;
    for (int i = 0;i< pointcloud_data.size(); i++)
    {
        new_data = pointcloud_data[i];

        update_arr(new_data);

        if (process_arr_.size() == BUFFER_SIZE)
        {
            float score = 1.f;
            ret_val = outlier_removal(score);
            pointcloud_data[i - floor(BUFFER_SIZE/2)].nOutlierFlag = ret_val;
            pointcloud_data[i - floor(BUFFER_SIZE / 2)].min_score = score;
        }
    }

    //isolate removal
    for (int i = 1; i < pointcloud_data.size() - 1; i++)
    {
        Point2fData prev_data = pointcloud_data[i - 1];
        Point2fData curr_data = pointcloud_data[i];
        Point2fData next_data = pointcloud_data[i + 1];
        if (curr_data.nOutlierFlag != OUTLIERFLAG::LEVEL3)
            continue;
        float isolate_dist_thr = curr_data.y * 0.05 + 6;
        float dist_diff1 = abs(curr_data.y - prev_data.y);
        float dist_diff2 = abs(curr_data.y - next_data.y);
        if ((dist_diff1 > isolate_dist_thr || prev_data.nOutlierFlag == OUTLIERFLAG::LEVEL1)  &&
            (dist_diff2 > isolate_dist_thr || next_data.nOutlierFlag == OUTLIERFLAG::LEVEL1))
            pointcloud_data[i].nOutlierFlag= OUTLIERFLAG::LEVEL2;
        else
        {
            int is = i - 5;
            if (is < 0) is = 0;
            int ie = is + 10;
            if (ie > pointcloud_data.size())
            {
                ie = pointcloud_data.size();
                is = ie - 10;
            }
            int found_cnt = 0;
            for (int j = is; j < ie; j++)
                if (OUTLIERFLAG::LEVEL1 == pointcloud_data[j].nOutlierFlag) found_cnt++;
            if (found_cnt > 1 && curr_data.min_score < 0.9)
                pointcloud_data[i].nOutlierFlag = OUTLIERFLAG::LEVEL2;
        }
    }
    //
}

void T2OutlierRemoval::update_arr(Point2fData &newdata)
{
    if (process_arr_.size() < BUFFER_SIZE)
    {
        process_arr_.push_back(newdata);
    }
    else
    {
        process_arr_.erase(process_arr_.begin());
        process_arr_.push_back(newdata);
    }
}

void T2OutlierRemoval::calculate_dist_eu(uint8_t ids, uint8_t ide, float dist_ratio, float& mean_dist, uint8_t& cnt, std::vector<Point2fData>& supports)
{
    cnt = 0;
    mean_dist = 0;
    Point2fData curr_data, tmp_data;
    for (uint8_t i = ids; i <= ide; i++)
    {
        uint8_t ncount = 0;
        float dist_eu = 0;
        curr_data = process_arr_[i];
        if (curr_data.y < 100)
            continue;
        for (uint8_t j = i - 4; j <= i + 4; j++)
        {
            if (j == i)
                continue;
            tmp_data = process_arr_[j];
            if ( abs(curr_data.y - tmp_data.y) > curr_data.y * 0.2 || tmp_data.y < 100 )
                continue;
            dist_eu = dist_eu + abs( tmp_data.y- curr_data.y );
            ncount = ncount + 1;
        }

        if (ncount > 4)
            dist_eu = dist_eu / float(ncount);
        else
            dist_eu = 0;
        float delta_thr = process_arr_[i].y * dist_ratio;
        if ( dist_eu < delta_thr && ncount > 4 )
        {
            cnt = cnt + 1;
            mean_dist = mean_dist + process_arr_[i].y;
            supports.push_back(process_arr_[i]);
        }
    }
    if (cnt > 0)
        mean_dist = mean_dist / float(cnt);
}

uint8_t T2OutlierRemoval::calc_cossim(const Point2fData &point_data,const std::vector<Point2fData> &supports, float& min_score,int& ft_cnt)
{
    ft_cnt = 0;
    min_score = 1;
    float score_thr = 0.77;
    uint8_t support_cnt = 0;
    float x0 = point_data.fx - supports[0].fx;
    float y0 = point_data.fy - supports[0].fy;
    for (int i = 1; i < supports.size(); i++)
    {
        float x1 = point_data.fx - supports[i].fx;
        float y1 = point_data.fy - supports[i].fy;
        float n0 = x0 * x0 + y0 * y0;
        float n1 = x1 * x1 + y1 * y1;
        float nx = x0 * x1 + y0 * y1;
        float score = (nx * nx / (n0 * n1));
        if (score < score_thr)
            support_cnt++;
//        if (score > 0.92)
//            ft_cnt++;
        if (score < min_score)
            min_score = score;
    }
    return support_cnt;
}

bool T2OutlierRemoval::check_line(const std::vector<Point2fData> &supports)
{
    float AtA[4] = { 0 };
    float Atb[2] = { 0 };
    for (int i = 0; i < supports.size(); i++)
    {
        AtA[0] += supports[i].fx * supports[i].fx;
        AtA[1] += supports[i].fx;
        Atb[0] += supports[i].fx * supports[i].fy;
        Atb[1] += supports[i].fy;
    }
    AtA[2] = AtA[1];
    AtA[3] = supports.size();

    float AtA_ad[4] = { AtA[3], -AtA[2], -AtA[1], AtA[0] };
    float AtA_det = AtA[0] * AtA[3] - AtA[1] * AtA[2];
    if (fabs(AtA_det) < 1e-8)
        return true;
    float p1 = AtA_ad[0] / AtA_det * Atb[0] + AtA_ad[1] / AtA_det * Atb[1];
    float p2 = AtA_ad[2] / AtA_det * Atb[0] + AtA_ad[3] / AtA_det * Atb[1];

    int outlier_cnt = 0;
    float outlier_dist_thr = 10;
    for (int i = 0; i < supports.size(); i++)
    {
        float dist = p1 * supports[i].fx + p2 - supports[i].fy;
        if (fabs(dist) > outlier_dist_thr)
        {
            outlier_cnt++;
        }
    }
    return outlier_cnt > supports.size() * 0.5;
}

uint8_t T2OutlierRemoval::outlier_removal(float& min_score)
{
    min_score = 1.f;
    uint8_t flag = OUTLIERFLAG::NORMAL;
    Point2fData curr_data = process_arr_[floor(BUFFER_SIZE / 2)];
    if (curr_data.y > 2200)
        return flag;

    float ratio = 0.02;
    uint8_t cnt_right, cnt_left;
    float mean_dist_r, mean_dist_l;
    std::vector<Point2fData> support_left, support_right;
    calculate_dist_eu( 4,  14,  ratio, mean_dist_l, cnt_left, support_left);
    calculate_dist_eu( 16, 26, ratio, mean_dist_r, cnt_right, support_right);
    float diff_l = abs(curr_data.y - mean_dist_l);
    if (diff_l < curr_data.y * 0.1)
        diff_l = 1;
    else
        diff_l = 0;

    float diff_r = abs( curr_data.y - mean_dist_r );
    if (diff_r < curr_data.y * 0.1)
        diff_r = 1;
    else
        diff_r = 0;

    if ((cnt_left > 7  &&  diff_l > 0.5) || (cnt_right > 7 && diff_r > 0.5))
    {
        uint8_t ncount = 0;
        float dist_eu = 0;
        for (uint8_t j = 10; j < 21; j++)
        {
            Point2fData tmp_data;
            tmp_data = process_arr_[j];
            if (tmp_data.y < 100)
                continue;

            dist_eu = dist_eu + abs(tmp_data.y - curr_data.y);
            ncount = ncount + 1;
        }
        if (ncount > 5)
            dist_eu = dist_eu / float(ncount);
        else
            dist_eu = 0;
        float delta_thr = curr_data.y * 0.03;
        if (dist_eu > delta_thr&& ncount > 5)
        {
            flag = OUTLIERFLAG::LEVEL3;
            uint8_t support_cnt = 0;
            uint8_t sel_cnt = 0;
            int ft_cnt = 0;
            bool ret_val = false;
            if (diff_l > 0.5) {
                std::reverse(support_left.begin(), support_left.end());
                ret_val = check_line(support_left);
                support_cnt = calc_cossim(curr_data, support_left, min_score,ft_cnt);
                sel_cnt = support_left.size();
            }
            else {
                ret_val = check_line(support_right);
                support_cnt = calc_cossim(curr_data, support_right, min_score,ft_cnt);
                sel_cnt = support_right.size();
            }
//            if (curr_data.y < 1500)
//                ft_cnt = 3;
            if(support_cnt > 1 && sel_cnt > 4 && false == ret_val/*&& ft_cnt > 2*/)
                flag = OUTLIERFLAG::LEVEL1;
        }
    }

    if (OUTLIERFLAG::NORMAL == flag)
    {
        float isolate_dist_thr = curr_data.y * 0.05 + 6;
        float dist_diff1 = abs(curr_data.y - process_arr_[floor(BUFFER_SIZE / 2) - 1].y);
        float dist_diff2 = abs(curr_data.y - process_arr_[floor(BUFFER_SIZE / 2) + 1].y);
        if (dist_diff1 > isolate_dist_thr && dist_diff2 > isolate_dist_thr)
            flag = OUTLIERFLAG::LEVEL1;
    }
    return flag;
}

int T2OutlierRemoval::smooth_4points_once(vector<Point2fData> &data,const float fMaxDiff,const float fMaxBiasThr,const uint8_t nKernelSize,const double dbDistance)
{
    int nRet = -1;
    if (data.size() < 1)
        return nRet;
    float fMinAdiff = { 12000 };
    float fMean = { 0 };
    int nMinDiffIndex1 = { 0 }, nMinDiffIndex2 = { 0 };
    int i = { 0 }, j = { 0 };
    double dbMeanTotal = { 0 };
    uint8_t  nMeanCount = { 0 };
    uint8_t  nOffset = {0};
    nOffset = (uint8_t)( ceil(nKernelSize / 2.0) - 1);
    for (int k = 0; k < data.size() - nKernelSize; k++)
    {
        if (data[k].fDist > dbDistance) //filter distance < 1m
            continue;
        fMinAdiff = 12000;
        for (j = 0; j < nKernelSize; j++) {
            for (i = (j + 1); i < nKernelSize; i++) {
                if (abs(data[k+i].fDist - data[k+j].fDist) < fMinAdiff) {
                    nMinDiffIndex1 =j;
                    nMinDiffIndex2 = i;
                    fMean = (data[k+i].fDist + data[k+j].fDist) / 2.0;
                    fMinAdiff = abs(data[k + i].fDist - data[k + j].fDist);
                }
            }
        }
        //
        if (fMinAdiff > fMaxDiff)
            continue;
        // data for smooth
        dbMeanTotal = 0;
        nMeanCount = 0;
        for (i = 0; i < nKernelSize; i++)
        {
            if ((i == nMinDiffIndex1) || (i == nMinDiffIndex2)) {
                dbMeanTotal += fMean;
                nMeanCount++;
            }
            else if (abs(data[k + i].fDist - fMean) <= fMaxBiasThr) {
                dbMeanTotal += data[k + i].fDist;
                nMeanCount++;
            }
        }
        //
        data[k+nOffset].y = (dbMeanTotal / nMeanCount);
        data[k+nOffset].bIsSmooth = true;
    }
    return 0;
}

void T2OutlierRemoval::smoth_pointCloud(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes, const float fMaxDiff, const float fMaxBiasThr,
                                        const uint8_t nKernelSize,const double dbDistance)
{
    if (sPointCloud.size() < 1)
        return;
    int nTimes = { nRepeatTimes };
    if (nRepeatTimes < 1 || nRepeatTimes > 10) {
        nTimes = 1;
    }
    for (int i = 0; i < nTimes; i++) {
        smooth_4points_once(sPointCloud, fMaxDiff/(i+1), fMaxBiasThr/(i+1), nKernelSize,dbDistance);
    }
}

void T2OutlierRemoval::weightSmooth_pointCloud(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes,const float fMaxGap,const float fMaxDiff,const double dbDistance)
{
    doSmoothProcessing(sPointCloud, nRepeatTimes,fMaxGap,fMaxDiff, m_sWeigthSeed,dbDistance);
}

bool T2OutlierRemoval::setWeightSmoothParameter(const int nKernelSize, const int nFitPoly)
{
    Matrix<double, Dynamic, Dynamic> E(nKernelSize, nKernelSize);     // 和MatrixXd一致
    E.fill(0);
    //至少2个以上奇数点、至少线性、点数一定要大于或等于多项式次数，否则无解
    if (nKernelSize < 2 || (nKernelSize % 2) != 1 || nFitPoly < 1 || nKernelSize < nFitPoly)
    {
        std::cout << "error!" << endl;
        return false;
    }
    Matrix<double, Dynamic, Dynamic> X(nKernelSize, nFitPoly);     // 和MatrixXd一致

    int half = nKernelSize / 2; //取整 如5个点取2
    vector<int>x;
    for (int i = -half; i <= half; i++)
    {
        x.push_back(i);
    }

    for (int i = 0; i < nKernelSize; i++)
    {
        for (int j = 0; j < nFitPoly; j++)
        {
            X(i, j) = pow(x[i], j);
            //cout << "X:" << X(i, j) << endl; //test ok
        }
    }

    E = X * (X.transpose() * X).inverse() * X.transpose();
    cout << "E:" << endl << E << endl; //test
    m_sWeigthSeed = E.row(half);
    return true;
}

bool T2OutlierRemoval::getGuassKernel(const int n)
{
    if (n < 5 || ((n % 2) == 0)) //输入奇数阶数，且至少为5阶以上
    {
        cout << "error!" << endl;
        return false;
    }
    m_sGuassKernel = VectorXd(n);
    m_sGuassKernel.fill(0);
    /*  求取kernel函数:
        f(x) = (1 / sqrt(2 * PI)) * exp(-pow(x, 2) / 2)
     */

    int half = n / 2;
    int nIndex = { 0 };
    for (int x = -half; x <= half; x++)
    {
        m_sGuassKernel[nIndex] = (1 / sqrt(2 * PI)) * exp(-pow(x, 2) / 2);
        nIndex++;
    }

    double sum = 0;
//    QString strOut = "";
    for (int j = 0; j < n; j++)
    {
        cout << "kernel " << j << ":" << m_sGuassKernel[j] << endl; //test
//        strOut += QString("%1 ").arg(m_sGuassKernel[j], 0, 'f', 12);
        sum += m_sGuassKernel[j];
    }
//    cout <<"["<< strOut.toStdString()<<"]" << endl;
    cout << "sum:" << sum << endl;
    return true;
}

bool T2OutlierRemoval::setGuassSmoothParameter(const int nKernelSize)
{
    return getGuassKernel(nKernelSize);
}

void T2OutlierRemoval::guassSmooth_pointCloud(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes, const float fMaxGap,const float fMaxDiff,const double dbDistance)
{
    doSmoothProcessing(sPointCloud, nRepeatTimes,fMaxGap, fMaxDiff, m_sGuassKernel,dbDistance);
}

void T2OutlierRemoval::doSmoothProcessing(std::vector<Point2fData> & sPointCloud, const int nRepeatTimes, const float fMaxGap, const float fMaxDiff, const VectorXd &sWeight,const double dbDistance)
{
    if (sPointCloud.size() < 1)
        return;
    int nTimes = { nRepeatTimes };
    if (nRepeatTimes < 1 || nRepeatTimes > 10) {
        nTimes = 1;
    }
    int nMid = sWeight.size()/2;
    int nHeadTail = { 0 };
    float fMinDiff = { 17000 };
    for (int k = 0; k < nTimes; k++) {
        for (int i = 0; i < sPointCloud.size(); i++) {
            if (sPointCloud[i].fDist > dbDistance) //filter distance < 1m
                continue;
            sPointCloud[i].y = 0.0;
            int nNearestLeftIndex = -1;
            int nNearestRightIndex = -1;
            float dbNearset =2*dbDistance;
            float dbMinNearest =2*dbDistance;
            float fMinDiffLeft = 0;
            float fMinNearstDist = 0;
            if ((nMid <= i) &&(i < (sPointCloud.size() - nMid))) //除去首尾数据直接使用权重进行计算
            {
                dbMinNearest =2*dbDistance;
                for (int j = -nMid; j <= nMid; j++)
                {
                    for (int k = -nMid; k <= nMid; k++) {
                        if (j != k) {
                            dbNearset = abs(sPointCloud[j + i].fDist - sPointCloud[i + k].fDist);
                            if (dbMinNearest > dbNearset) {
                                dbMinNearest = dbNearset;
//                                if (j > k) {
                                nNearestRightIndex = j;
                                nNearestLeftIndex = k;
//                                }
//                                else {
//                                    nNearestRightIndex = k;
//                                    nNearestLeftIndex = j;
//                                }
                            }
                        }
                    }
                }
                for (int j = -nMid; j <= nMid; j++)
                {
                    fMinDiff = abs(sPointCloud[i+nNearestRightIndex].fDist - sPointCloud[j + i].fDist);
                    fMinDiffLeft = abs(sPointCloud[i+nNearestLeftIndex].fDist - sPointCloud[j + i].fDist);
                    if (fMinDiff < fMinDiffLeft) {
                        fMinNearstDist = fMinDiff;
                    }
                    else {
                        fMinNearstDist = fMinDiffLeft;
                    }
                    if (fMinNearstDist > fMaxDiff) {
                        sPointCloud[i].y = sPointCloud[i].fDist;
                        break;
                    }
                    else if (fMinNearstDist > fMaxGap) {
                        if (abs(nNearestLeftIndex - j) > abs(nNearestRightIndex - j)) {
                            sPointCloud[i].y += sWeight(j + nMid) * sPointCloud[nNearestRightIndex + i].fDist;
                        }
                        else {
                            sPointCloud[i].y += sWeight(j + nMid) * sPointCloud[nNearestLeftIndex + i].fDist;
                        }
                    }
                    else {
                        sPointCloud[i].y = sPointCloud[i].y + sWeight(j + nMid) * sPointCloud[j + i].fDist;
                    }
                }
            }
            else //首尾数据加权求和, 对数据首尾相连，形成环状
            {
                dbMinNearest =2*dbDistance;
                for (int j = -nMid; j <= nMid; j++)
                {
                    for (int k = -nMid; k <= nMid; k++) {
                        if (j != k) {
                            dbNearset = abs(sPointCloud[(j + i + sPointCloud.size()) % sPointCloud.size()].fDist - sPointCloud[(k + i + sPointCloud.size()) % sPointCloud.size()].fDist);
                            if (dbMinNearest > dbNearset) {
                                dbMinNearest = dbNearset;
//                                if (j > k) {
                                nNearestRightIndex = j;
                                nNearestLeftIndex = k;
//                                }
//                                else {
//                                    nNearestRightIndex = k;
//                                    nNearestLeftIndex = j;
//                                }
                            }
                        }
                    }
                }
                for (int j = -nMid; j <= nMid; j++)
                {
                    nHeadTail = (j + i + sPointCloud.size()) % sPointCloud.size();
                    fMinDiffLeft = abs(sPointCloud[i].fDist - sPointCloud[(nNearestLeftIndex + i + sPointCloud.size()) % sPointCloud.size()].fDist);
                    fMinDiff = abs(sPointCloud[i].fDist - sPointCloud[(nNearestRightIndex + i + sPointCloud.size()) % sPointCloud.size()].fDist);
                    if (fMinDiff < fMinDiffLeft) {
                        fMinNearstDist = fMinDiff;
                    }
                    else {
                        fMinNearstDist = fMinDiffLeft;
                    }
                    if (fMinNearstDist > fMaxDiff) {
                        sPointCloud[i].y = sPointCloud[i].fDist;
                        break;
                    }
                    else if (fMinNearstDist > fMaxGap) {
                        if (abs(nNearestLeftIndex - j) > abs(nNearestRightIndex - j)) {
                            sPointCloud[i].y += sWeight(j + nMid) * sPointCloud[(nNearestRightIndex + i + sPointCloud.size()) % sPointCloud.size()].fDist;
                        }
                        else {
                            sPointCloud[i].y += sWeight(j + nMid) * sPointCloud[(nNearestLeftIndex + i + sPointCloud.size()) % sPointCloud.size()].fDist;
                        }
                    }
                    else {
                        sPointCloud[i].y = sPointCloud[i].y + sWeight(j + nMid) * sPointCloud[nHeadTail].fDist;
                    }
                }
            }
        }
    }
}


void smoothPointCloud(LstPointCloud &pointCloud, int iSmoothMode, T2OutlierRemoval &t2removal) {
    if (iSmoothMode == 0) return;
    std::vector<Point2fData> data;
    for (auto point: pointCloud) {
        Point2fData tmp;
        tmp.x = point.dAngle;
        tmp.y = point.u16Dist;
        tmp.fx = tmp.y * cos(tmp.x / 180.0 * PI);
        tmp.fy = tmp.y * sin(tmp.x / 180.0 * PI);
        tmp.fDist = tmp.y;
        tmp.fRawDistance = point.u16Dist;
        tmp.bIsInVaid = point.bValid;
        tmp.fSpeed = point.u16Speed;
        tmp.fRawAngle = point.dAngleRaw;
        data.push_back(tmp);
    }
    switch (iSmoothMode) {
        case 1:
            t2removal.smoth_pointCloud(data, smoothParam.RepeatTimes, smoothParam.MaxDiff, smoothParam.MaxBiasThreshold,
                                       smoothParam.KernelSize, smoothParam.MaxFilterDistance);
            break;
        case 2:
            t2removal.weightSmooth_pointCloud(data, smoothParam.RepeatTimes, smoothParam.MaxBiasThreshold,
                                              smoothParam.MaxDiff, smoothParam.MaxFilterDistance);
            break;
        case 3:
            t2removal.guassSmooth_pointCloud(data, smoothParam.RepeatTimes, smoothParam.MaxBiasThreshold,
                                              smoothParam.MaxDiff, smoothParam.MaxFilterDistance);
            break;
        default:
            break;
    }
    pointCloud.clear();
    for (auto point2: data) {
        tsPointCloud point;
        point.u16Speed = point2.fSpeed;
        point.bValid = point2.bIsInVaid;
        point.u16Dist = point2.y;
        point.dAngle = point2.x;
        point.u16DistRaw = point2.fRawDistance;
        point.dAngleRaw = point2.fRawAngle;
        pointCloud.push_back(point);
    }
}
