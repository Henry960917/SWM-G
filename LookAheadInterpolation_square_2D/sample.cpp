/*****************************************************************************/
/* FILE        : LookAheadMotion.cpp                                         */
/* DESCRIPTION : Samples that perform path interporation with look ahead.    */
/*****************************************************************************/

/*****************************************************************************/
/* Header                                                                    */
/*****************************************************************************/
#include "SSCApi.h"
#include "AdvancedMotionApi.h"
#include <stdio.h>
#include <cmath>

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
using namespace sscApi;
using namespace std;

/*****************************************************************************/
/* Global valiables                                                          */
/*****************************************************************************/
SSCApi          sscLib;
CoreMotionStatus CmStatus;
CoreMotion       sscLib_cm(&sscLib);
AdvancedMotion   sscLib_Adv(&sscLib);

/*****************************************************************************/
/* Prototype declaration                                                     */
/*****************************************************************************/
void Initialize();
void Finalize();

/*****************************************************************************/
/* Function                                                                  */
/*****************************************************************************/

/*---------------------------------------------------------------------------*/
/* Function    : _tmain                                                      */
/* Description : Main Function.                                              */
/*---------------------------------------------------------------------------*/
int _tmain(int argc, _TCHAR* argv[])
{
    printf("Program Start\n");
    Sleep(1000);

    Initialize();

    // LookAhead Start --------------------------------------------------------
    printf("LookAhead Interpolation\n");

    // Create LookAhead Buffer.
    sscLib_Adv.advMotion->CreatePathIntplLookaheadBuffer(0, 1000);

    // 清空該緩衝區內所有舊的、殘留的路徑指令
    sscLib_Adv.advMotion->ClearPathIntplLookahead(0);

    // 宣告並初始化 LookAhead 的總體配置
    AdvMotion::PathIntplLookaheadConfiguration conf = AdvMotion::PathIntplLookaheadConfiguration();
    conf.axisCount = 2;
    conf.axis[0] = 0;
    conf.axis[1] = 1;
    conf.compositeVel = 10000;
    conf.compositeAcc = 20000;
    conf.sampleDistance = 1000; // 前瞻演算法的取樣距離（用來計算轉角速度衰減的物理參數）

    // 將填好的全域環境配置寫入 0 號 LookAhead 控制器
    sscLib_Adv.advMotion->SetPathIntplLookaheadConfiguration(0, &conf);
    // 宣告並初始化 LookAhead 路徑點指令
    AdvMotion::PathIntplLookaheadCommand path = AdvMotion::PathIntplLookaheadCommand();
    path.numPoints = 4;

    //(0,0)->(10000,0)
    path.point[0].type = AdvMotion::PathIntplLookaheadSegmentType::Linear;
    path.point[0].data.linear.axisCount = 2;
    path.point[0].data.linear.axis[0] = 0;
    path.point[0].data.linear.axis[1] = 1;
    path.point[0].data.linear.target[0] = 10000;
    path.point[0].data.linear.target[1] = 0;
    path.point[0].smoothRadius = 1000;//在此點轉彎處的過渡圓弧半徑

	//(10000,0)->(10000,10000)
    path.point[1].type = AdvMotion::PathIntplLookaheadSegmentType::Linear;
    path.point[1].data.linear.axisCount = 2;
    path.point[1].data.linear.axis[0] = 0;
    path.point[1].data.linear.axis[1] = 1;
    path.point[1].data.linear.target[0] = 10000;
    path.point[1].data.linear.target[1] = 10000;
    path.point[1].smoothRadius = 1000;

	//(10000,10000)->(0,10000)
    path.point[2].type = AdvMotion::PathIntplLookaheadSegmentType::Linear;
    path.point[2].data.linear.axisCount = 2;
    path.point[2].data.linear.axis[0] = 0;
    path.point[2].data.linear.axis[1] = 1;
    path.point[2].data.linear.target[0] = 0;
    path.point[2].data.linear.target[1] = 10000;
    path.point[2].smoothRadius = 1000;

	//(0,10000)->(0,0)
    path.point[3].type = AdvMotion::PathIntplLookaheadSegmentType::Linear;
    path.point[3].data.linear.axisCount = 2;
    path.point[3].data.linear.axis[0] = 0;
    path.point[3].data.linear.axis[1] = 1;
    path.point[3].data.linear.target[0] = 0;
    path.point[3].data.linear.target[1] = 0;

    sscLib_Adv.advMotion->AddPathIntplLookaheadCommand(0, &path);

    sscLib_Adv.advMotion->StartPathIntplLookahead(0);

    Sleep(5000);

    sscLib_Adv.advMotion->StopPathIntplLookahead(0);
 
    Finalize();

    printf("Program End\n");
    Sleep(3000);
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Function    : Initialize                                                  */
/* Description : Device creation ~ Run until Homing.                         */
/*---------------------------------------------------------------------------*/
void Initialize()
{
    // Create devices.
    char path[] = "C:\\Program Files\\MotionSoftware\\SWM-G\\";
    sscLib.CreateDevice(path,
        DeviceType::DeviceTypeNormal,
        INFINITE);

	char deviceName[] = "LookAheadMotion";
    sscLib.SetDeviceName(deviceName);

    sscLib.StartCommunication(INFINITE);

    sscLib_cm.axisControl->SetServoOn(0, 1);
    sscLib_cm.axisControl->SetServoOn(1, 1);

    while (true)
    {
        sscLib_cm.GetStatus(&CmStatus);
        if (CmStatus.axesStatus[0].servoOn &&
            CmStatus.axesStatus[1].servoOn)
        {
            break;
        }

        Sleep(100);
    }

    // Homing.
    for (int i = 0; i < 2; i++)
    {
        Config::HomeParam homeParam;
        sscLib_cm.config->GetHomeParam(i, &homeParam);
        homeParam.homeType = Config::HomeType::CurrentPos;
        sscLib_cm.config->SetHomeParam(i, &homeParam);
        sscLib_cm.home->StartHome(i);
        sscLib_cm.motion->Wait(i);
    }
}

/*---------------------------------------------------------------------------*/
/* Function    : Finalize                                                    */
/* Description : It executes from servo OFF to device close.                 */
/*---------------------------------------------------------------------------*/
void Finalize()
{
    // Set servo off.
    sscLib_cm.axisControl->SetServoOn(0, 0);
    sscLib_cm.axisControl->SetServoOn(1, 0);

    while (true)
    {
        sscLib_cm.GetStatus(&CmStatus);
        if (!CmStatus.axesStatus[0].servoOn &&
            !CmStatus.axesStatus[1].servoOn)
        {
            break;
        }

        Sleep(100);
    }

    // Stop Communication.
    sscLib.StopCommunication(INFINITE);

    //close device.
    sscLib.CloseDevice();
}
