#include "AdvancedMotionApi.h"
#include "CompensationApi.h"
#include "CoreMotionApi.h"
#include "EventApi.h"
#include "IOApi.h"
#include "LogApi.h"
#include "UserMemoryApi.h"
#include "SSCApi.h"
#include <stdio.h>
#include <math.h>

using namespace sscApi;
using namespace std;

SSCApi sscLib; // 類別:SSCApi, 物件:sscLib (通訊用)(裝置管理員)
CoreMotionStatus CmStatus; // 類別:CoreMotionStatus, 物件:CmStatus (馬達狀態用)
CoreMotion sscLib_Cm(&sscLib); //類別:CoreMotion, 物件:sscLib_cm (馬達控制用)

void ini();
void final();

int main() {
	Motion::LinearIntplCommand linearIntplCmd = Motion::LinearIntplCommand();
	printf("PROGRAM START\n");
	Sleep(1000);
	ini();
	linearIntplCmd.axisCount = 3; // 共3軸參與插補
	linearIntplCmd.axis[0] = 0; // 第一軸X
	linearIntplCmd.axis[1] = 1; // 第二軸Y
	linearIntplCmd.axis[2] = 2; // 第三軸Z
	linearIntplCmd.target[0] = 100000; // X軸目標位置
	linearIntplCmd.target[1] = 200000; // Y軸目標位置
	linearIntplCmd.target[2] = 300000; // Z軸目標位置
	linearIntplCmd.profile.type = ProfileType::Trapezoidal;
	linearIntplCmd.profile.velocity = 100000;
	linearIntplCmd.profile.acc = 1000000;
	linearIntplCmd.profile.dec = 1000000;
	sscLib_Cm.motion->StartLinearIntplPos(&linearIntplCmd);

	sscLib_Cm.motion->Wait(0);
	sscLib_Cm.motion->Wait(1);
	sscLib_Cm.motion->Wait(2);

	final();
	printf("PROGRAM END\n");
	Sleep(3000);
	return 0;
}

void ini() {
	//創device
	char path[] = "C:\\Program Files\\MotionSoftware\\SWM-G\\";
	sscLib.CreateDevice(path,DeviceType::T::DeviceTypeNormal, INFINITE);
	char deviceName[] = "LinearInterpolation_3D";
	sscLib.SetDeviceName(deviceName);

	//開始通訊
	sscLib.StartCommunication(INFINITE);

	//使能
	sscLib_Cm.axisControl->SetServoOn(0, 1);
	sscLib_Cm.axisControl->SetServoOn(1, 1);
	sscLib_Cm.axisControl->SetServoOn(2, 1);
	while (true) {
		sscLib_Cm.GetStatus(&CmStatus);
		if (CmStatus.axesStatus[0].servoOn && CmStatus.axesStatus[1].servoOn && 
			CmStatus.axesStatus[2].servoOn) {
			break;
		}
		Sleep(100);
	}

	//回原點
	for (int i = 0; i < 3; i++) {
		Config::HomeParam homeParam;
		sscLib_Cm.config->GetHomeParam(i, &homeParam);
		homeParam.homeType = Config::HomeType::CurrentPos;
		sscLib_Cm.config->SetHomeParam(i, &homeParam);
		sscLib_Cm.home->StartHome(i);
	}
	sscLib_Cm.motion->Wait(0);
	sscLib_Cm.motion->Wait(1);
	sscLib_Cm.motion->Wait(2);
}

void final() {
	// 斷電
	sscLib_Cm.axisControl->SetServoOn(0, 0);
	sscLib_Cm.axisControl->SetServoOn(1, 0);
	sscLib_Cm.axisControl->SetServoOn(2, 0);
	while (true) {
		sscLib_Cm.GetStatus(&CmStatus);
		if (!CmStatus.axesStatus[0].servoOn && !CmStatus.axesStatus[1].servoOn &&
			!CmStatus.axesStatus[2].servoOn) {
			break;
		}
		Sleep(100);
	}

	//關閉通訊
	sscLib.StopCommunication(INFINITE);

	//關device
	sscLib.CloseDevice();
}
