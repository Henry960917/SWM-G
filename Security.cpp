#include "AdvancedMotionApi.h"
#include "CompensationApi.h"
#include "CoreMotionApi.h"
#include "EventApi.h"
#include "IOApi.h"
#include "LogApi.h"
#include "UserMemoryApi.h"
#include "SSCApi.h"
#include <iostream>

#define OP_STATE(state) \
    ((state == OperationState::Idle)          ? "Idle"          : \
     (state == OperationState::Pos)           ? "Pos"           : \
     (state == OperationState::Jog)           ? "Jog"           : \
     (state == OperationState::Home)          ? "Home"          : \
     (state == OperationState::Sync)          ? "Sync"          : \
     (state == OperationState::GantryHome)    ? "GantryHome"    : \
     (state == OperationState::Stop)          ? "Stop"          : \
     (state == OperationState::Intpl)         ? "Intpl"         : \
     (state == OperationState::ConstLinearVelocity) ? \
                                    "ConstLinearVelocity"       : \
     (state == OperationState::Trq)           ? "Trq"           : \
     (state == OperationState::DirectControl)       ? \
                                    "DirectControl"             : \
     (state == OperationState::PVT)           ? "PVT"           : \
     (state == OperationState::ECAM)          ? "ECAM"          : \
     (state == OperationState::SyncCatchUp)   ? "SyncCatchUp"   : \
     (state == OperationState::DancerControl) ? "DancerControl" : \
     "")


using namespace std;
using namespace sscApi;

int main(){
	SSCApi sscLib; // 類別:SSCApi, 物件:sscLib (通訊用)(裝置管理員)
	CoreMotionStatus CmStatus; // 類別:CoreMotionStatus, 物件:CmStatus (馬達狀態用)
	CoreMotion sscLib_Cm(&sscLib); //類別:CoreMotion, 物件:sscLib_cm (馬達控制用)
	printf("PROGRAM START\n");
	Sleep(1000);

	//創device
	char path[] = "C:\\Program Files\\MotionSoftware\\SWM-G\\";
	sscLib.CreateDevice(path, DeviceType::T::DeviceTypeNormal, INFINITE);
	char deviceName[] = "Security";
	sscLib.SetDeviceName(deviceName);

	//開始通訊
	sscLib.StartCommunication(INFINITE);

	//使能
	sscLib_Cm.axisControl->SetServoOn(0, 1);
	while (true){
		sscLib_Cm.GetStatus(&CmStatus);
		if (CmStatus.axesStatus[0].servoOn) {
			break;
		}
		Sleep(50);
	}

	//回原點
	Config::HomeParam homeParam;
	sscLib_Cm.config->GetHomeParam(0, &homeParam);
	homeParam.homeType = Config::HomeType::CurrentPos;
	sscLib_Cm.config->SetHomeParam(0, &homeParam);
	sscLib_Cm.home->StartHome(0);
	sscLib_Cm.motion->Wait(0);

	//Jog
	Motion::JogCommand jogcmd = Motion::JogCommand();
	jogcmd.profile.type = ProfileType::Trapezoidal;
	jogcmd.axis = 0;
	jogcmd.profile.velocity = 2500000;
	jogcmd.profile.acc = 100000;
	jogcmd.profile.dec = 100000;
	sscLib_Cm.motion->StartJog(&jogcmd);

	CoreMotionAxisStatus *cmAxis;
	cmAxis = &CmStatus.axesStatus[0];

	printf("We will begin checking the motor.\n");
	printf("If there are problems with it, we will stop it immediately.\n");

	while(true){
		// 取得目前的狀態
		sscLib_Cm.GetStatus(&CmStatus);
		if(cmAxis->actualVelocity > 200000){
			//減速到停
			printf("The motor is TOO FAST!!! We will stop it.\n");
			sscLib_Cm.motion->Stop(0, 10000);
			break;
		}
		else if (cmAxis->actualTorque>80.0) {
			//減速到停
			printf("The motor is TOO STRONG!!! We will stop it.\n");
			sscLib_Cm.motion->Stop(0, 10000);
			break;
		}
		else if (cmAxis->ampAlarm==1) {
			//給他急停
			printf("The motor has an AMP ALARM!!! We will stop it.\n");
			sscLib_Cm.motion->ExecQuickStop(0);
			break;
		}
		else if (cmAxis->negativeLS==1) {
			//給他急停
			printf("The motor has hit the NEGATIVE LIMIT!!! We will stop it.\n");
			sscLib_Cm.motion->ExecQuickStop(0);
			break;
		}
		else if (cmAxis->positiveLS==1) {
			//給他急停
			printf("The motor has hit the POSITIVE LIMIT!!! We will stop it.\n");
			sscLib_Cm.motion->ExecQuickStop(0);
			break;
		}
	}

	// 等待軸停
	sscLib_Cm.motion->Wait(0);

	// 斷電
	sscLib_Cm.axisControl->SetServoOn(0, 0);
	while (true) {
		sscLib_Cm.GetStatus(&CmStatus);
		if (!CmStatus.axesStatus[0].servoOn) {
			break;
		}
	}

	sscLib.StopCommunication(INFINITE);
	sscLib.CloseDevice();
	printf("Everything is done, and the program will end in 3 seconds.\n");
	Sleep(3000);
	printf("PROGRAM END\n");
    return 0;
}
