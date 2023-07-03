#ifndef _HCSDK_H_
#define _HCSDK_H_

#include "HcData.h"

#ifdef WIN_DLL
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

    DLL_EXPORT bool hcSDKInitialize(const char* chPort, const char* chLidarModel, int iBaud, int iReadTimeoutMs, bool bDistQ2, bool bGetLoopData, bool bPollMode);

    DLL_EXPORT bool hcSDKUnInit();

	//set callback function for error code
    DLL_EXPORT void setSDKCallBackFunErrorCode(CallBackFunErroCode fun);

	//set callback function for Statistic infomation
    DLL_EXPORT void setSDKCallBackFunSecondInfo(CallBackFunSecondInfo fun);

	//set callback function for rx pointclouds
    DLL_EXPORT void setSDKCallBackFunPointCloud(CallBackFunPointCloud fun);

	//get error code
    DLL_EXPORT int getSDKLastErrCode();

    DLL_EXPORT bool getSDKLidarInfo();

	//get SDK status
    DLL_EXPORT int getSDKStatus();

	// get SDK Version
    DLL_EXPORT char* getSDKVersion();

	//get lidar ID
    DLL_EXPORT char* getSDKLidarID();

	// get factory infomation
    DLL_EXPORT char* getSDKFactoryInfo();

	//get lidar model
    DLL_EXPORT char* getSDKLidarModel();

	//get firmware version
    DLL_EXPORT char* getSDKFirmwareVersion();

	//get hardware version
    DLL_EXPORT char* getSDKHardwareVersion();

	// set work parameter
    DLL_EXPORT void setSDKWorkPara(tsSDKPara& sSDKPara);

	//poll mode,get pointclouds
    DLL_EXPORT bool getSDKRxPointClouds(LstPointCloud& lstG);

	//Factory mode for test
    DLL_EXPORT void setSDKFactoryMode(bool bFactoryMode = true);

	//start the factory mode 
    DLL_EXPORT bool startSDKFactoryModeRun();
	
	//set one circle data to output 
    DLL_EXPORT void setSDKCircleDataMode();

	//set the lidar status is powered
    DLL_EXPORT void setSDKLidarPowerOn(bool bPowerOn);

	//set the lidar low speed status
    DLL_EXPORT void setSDKLidarLowSpeed(bool bLow);

	//零度角补偿开关
    DLL_EXPORT void setSDKAngOffset(bool bAngOffSet);

	//杂点 射线过滤开关
	//void setSDKFilter(bool bfilter);

    DLL_EXPORT void setSDKPointCloudLattice(bool bDistinguish);

    DLL_EXPORT void setSDKD2M7SubComp(bool bComp);

    DLL_EXPORT void setSDKT2RotationalSpeed(int iRotationalSpeed);

    DLL_EXPORT void setSDKT2LidarStart(bool bStarted);

#ifdef __cplusplus
};
#endif

#endif

