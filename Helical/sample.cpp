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
	Motion::HelicalIntplCommand helicalCommand = Motion::HelicalIntplCommand();
	printf("PROGRAM START\n");
	Sleep(1000);
	ini();
	helicalCommand.helicalProfileType = Motion::HelicalIntplProfileType::Helical;
	helicalCommand.axis[0] = 0; // 第一軸X
	helicalCommand.axis[1] = 1; // 第二軸Y
	helicalCommand.zAxis = 2; // 第三軸Z
	helicalCommand.centerPos[0] = 50000 * sqrt(2.0); // 圓心x座標
	helicalCommand.centerPos[1] = 50000 * sqrt(2.0); // 圓心y座標
	helicalCommand.zEndPos = 500000; // z軸終點位置
	helicalCommand.clockwise = 0;
	helicalCommand.arcLengthDegree = 360*5; // 旋轉角度
	helicalCommand.profile.type = ProfileType::Trapezoidal;
	helicalCommand.profile.velocity = 100000;
	helicalCommand.profile.acc = 18000;
	helicalCommand.profile.dec = 18000;
	sscLib_Cm.motion->StartHelicalIntplPos(&helicalCommand);

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
	sscLib.CreateDevice(path, DeviceType::T::DeviceTypeNormal, INFINITE);
	char deviceName[] = "Helical Motion";
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
