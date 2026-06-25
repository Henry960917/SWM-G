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
	Motion::CenterAndLengthCircularIntplCommand circularIntplCmd = Motion::CenterAndLengthCircularIntplCommand();
	printf("PROGRAM START\n");
	Sleep(1000);
	ini();
	circularIntplCmd.axis[0] = 0; // 第一軸X
	circularIntplCmd.axis[1] = 1; // 第二軸Y
	circularIntplCmd.centerPos[0] = 100000; // 圓心x座標
	circularIntplCmd.centerPos[1] = 200000; // 圓心y座標
	circularIntplCmd.arcLengthDegree = 360; // 旋轉角度
	circularIntplCmd.profile.type = ProfileType::Trapezoidal;
	circularIntplCmd.profile.velocity = 100000;
	circularIntplCmd.profile.acc = 18000;
	circularIntplCmd.profile.dec = 18000;
	sscLib_Cm.motion->StartCircularIntplPos(&circularIntplCmd);

	sscLib_Cm.motion->Wait(0);
	sscLib_Cm.motion->Wait(1);

	final();
	printf("PROGRAM END\n");
	Sleep(3000);
	return 0;
}

void ini() {
	//創device
	char path[] = "C:\\Program Files\\MotionSoftware\\SWM-G\\";
	sscLib.CreateDevice(path,DeviceType::T::DeviceTypeNormal, INFINITE);
	char deviceName[] = "CircularInterpolation_3D";
	sscLib.SetDeviceName(deviceName);

	//開始通訊
	sscLib.StartCommunication(INFINITE);

	//使能
	sscLib_Cm.axisControl->SetServoOn(0, 1);
	sscLib_Cm.axisControl->SetServoOn(1, 1);
	while (true) {
		sscLib_Cm.GetStatus(&CmStatus);
		if (CmStatus.axesStatus[0].servoOn && CmStatus.axesStatus[1].servoOn) {
			break;
		}
		Sleep(100);
	}

	//回原點
	for (int i = 0; i < 2; i++) {
		Config::HomeParam homeParam;
		sscLib_Cm.config->GetHomeParam(i, &homeParam);
		homeParam.homeType = Config::HomeType::CurrentPos;
		sscLib_Cm.config->SetHomeParam(i, &homeParam);
		sscLib_Cm.home->StartHome(i);
	}
	sscLib_Cm.motion->Wait(0);
	sscLib_Cm.motion->Wait(1);
}

void final() {
	// 斷電
	sscLib_Cm.axisControl->SetServoOn(0, 0);
	sscLib_Cm.axisControl->SetServoOn(1, 0);
	while (true) {
		sscLib_Cm.GetStatus(&CmStatus);
		if (!CmStatus.axesStatus[0].servoOn && !CmStatus.axesStatus[1].servoOn) {
			break;
		}
		Sleep(100);
	}

	//關閉通訊
	sscLib.StopCommunication(INFINITE);

	//關device
	sscLib.CloseDevice();
}
