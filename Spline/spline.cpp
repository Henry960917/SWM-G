#include "SSCApi.h"
#include "AdvancedMotionApi.h"
#include <stdio.h>

using namespace sscApi;
using namespace std;

SSCApi          sscLib;
CoreMotionStatus CmStatus;
CoreMotion       sscLib_cm(&sscLib);
AdvancedMotion   sscLib_Adv(&sscLib);
const double PI = 3.1415926535897932384626433832795;

void Initialize();
void Finalize();

int main()
{
    printf("Program Start\n");
    Sleep(1000);

    Initialize();

	  // Buffer ID 0, 可容納100個點
    sscLib_Adv.advMotion->CreateSplineBuffer(0, 100);

    AdvMotion::TotalTimeSplineCommand splineCommand = AdvMotion::TotalTimeSplineCommand();
    splineCommand.dimensionCount = 2;
    splineCommand.axis[0] = 0;
    splineCommand.axis[1] = 1;
    splineCommand.totalTimeMilliseconds = 10000;

    // n個軌跡點的陣列
    AdvMotion::SplinePoint splinePoint[9];

    sscLib_cm.GetStatus(&CmStatus);

	// 把馬達現在的位置設定為這條軌跡的起點
    splinePoint[0].pos[0] = CmStatus.axesStatus[0].posCmd;
    splinePoint[0].pos[1] = CmStatus.axesStatus[1].posCmd;

    
    splinePoint[1].pos[0] = PI/2.0;
    splinePoint[1].pos[1] = 1;

    splinePoint[2].pos[0] = PI;
    splinePoint[2].pos[1] = 0;

    splinePoint[3].pos[0] = 3.0*PI/2.0;
    splinePoint[3].pos[1] = -1;

    splinePoint[4].pos[0] = 2.0*PI;
    splinePoint[4].pos[1] = 0;

    splinePoint[5].pos[0] = 5.0*PI/2.0;
    splinePoint[5].pos[1] = 1;

    splinePoint[6].pos[0] = 3.0*PI;
    splinePoint[6].pos[1] = 0;

    splinePoint[7].pos[0] = 7.0*PI/2.0;
    splinePoint[7].pos[1] = -1;

    splinePoint[8].pos[0] = 4.0*PI;
    splinePoint[8].pos[1] = 0;
  
    sscLib_Adv.advMotion->StartCSplinePos(0, &splineCommand, 9, splinePoint);
    sscLib_cm.motion->Wait(0);
    sscLib_cm.motion->Wait(1);

    Finalize();

    printf("Program End\n");
    Sleep(3000);
    return 0;
}
