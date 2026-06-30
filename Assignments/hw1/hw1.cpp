/*****************************************************************************/
/* FILE        : TriggerMotion.cpp                                           */
/* DESCRIPTION : Sample to set the trigger and move the axis.                */
/*****************************************************************************/

/*****************************************************************************/
/* Header                                                                    */
/*****************************************************************************/
#include "SSCApi.h"
#include "CoreMotionApi.h"
#include <stdio.h>

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

    // Move the motor to the specified position.
    Motion::PosCommand posCommand = Motion::PosCommand();
    posCommand.profile.type = ProfileType::Trapezoidal;
    posCommand.axis = 0;
    posCommand.target = 1000000;
    posCommand.profile.velocity = 100000;
    posCommand.profile.acc = 50000;
    posCommand.profile.dec = 50000;

    // 0軸一邊跑，程式一邊往下
    sscLib_cm.motion->StartMov(&posCommand);

    // Execute "command with trigger".
    Motion::TriggerPosCommand tgrPosCommand = Motion::TriggerPosCommand();// 建立帶觸發功能的運動物件
    tgrPosCommand.profile.type = ProfileType::Trapezoidal;
    tgrPosCommand.axis = 1;
    tgrPosCommand.target = 500000;
    tgrPosCommand.profile.velocity = 100000;
    tgrPosCommand.profile.acc = 50000;
    tgrPosCommand.profile.dec = 50000;

    //----------------------------------------------------------------------------
    // Create Trigger.
    // ->Start when the remaining distance of 0 axis reaches 10000 pulse.
    //----------------------------------------------------------------------------
    Trigger trigger = Trigger(); // 建立觸發物件
    trigger.triggerAxis = 0; // 監聽0軸
    trigger.triggerType = TriggerType::RemainingDistance; // 觸發類型為剩餘距離(小於等於)
    trigger.triggerValue = 600000;// 觸發值
    tgrPosCommand.trigger = trigger; // 將觸發物件放入帶觸發功能的運動物件中

    sscLib_cm.motion->StartMov(&tgrPosCommand);

    // Wait for the 0 axis to run.
    // 確保0軸開始跑再往下執行
    Motion::WaitCondition waitCondition = Motion::WaitCondition();
    waitCondition.axisCount = 1; // 監聽1個軸
    waitCondition.axis[0] = 0; // 監聽0軸
    waitCondition.waitConditionType = Motion::WaitConditionType::MotionStarted; // 條件：運動已啟動
    sscLib_cm.motion->Wait(&waitCondition); // 等待0軸開始運動

    // Wait for 0 axis and 1 axis to stop.
    sscLib_cm.motion->Wait(0);
    sscLib_cm.motion->Wait(1);

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
        DeviceType::T::DeviceTypeNormal,
        INFINITE);

    // Set Device Name.
    char deviceName[] = "TriggerMotion";
    sscLib.SetDeviceName(deviceName);

    // Start Communication.
    sscLib.StartCommunication(INFINITE);

    // Set servo on.
    sscLib_cm.axisControl->SetServoOn(0, 1);
    sscLib_cm.axisControl->SetServoOn(1, 1);
    while (true)
    {
        sscLib_cm.GetStatus(&CmStatus);
        if (CmStatus.axesStatus[0].servoOn && CmStatus.axesStatus[1].servoOn)
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
    }

    sscLib_cm.motion->Wait(0);
    sscLib_cm.motion->Wait(1);
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
        if (!CmStatus.axesStatus[0].servoOn && !CmStatus.axesStatus[1].servoOn)
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
