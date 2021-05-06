#ifndef BASKETBALLINFO_H
#define BASKETBALLINFO_H

#include <math.h>

//loadparameter.cpp
//29.ini calculate RobotHeight
#define Deg2Rad 0.017453292 // pi/180
#define L_Calf 10.4255
#define L_Thigh 10.22566
#define L_BodyError 6.104489964
#define L_Body 23.273966890
#define L_Foot 2.256
#define L_Shoes 0.2
#define HalfBasketHeight 35.3

//strategy_main.cpp
//ContinuousStep
#define ContinuousMode 19
//Triangulation
#define Scale2Deg 0.087890625 // 360/4096
#define CameraHeight 3.925
//Object Color Models
#define Ballfarsize 500
#define Basketfarsize 1200
//HeadVerticalState
#define HeadTurnNear 0
#define HeadTurnClose 1
//HeadHorizontalState
#define HeadTurnRight 0
#define HeadTurnLeft 1
//For Calculate ErrorHorizontalAngle & ErrorVerticalAngle
#define RobotVisionWidth 320
#define RobotVisionHeight 240
//For MoveHead ErrorHorizontalAngle & ErrorVerticalAngle 
#define TraceDegreePercent 0.25
#define Deg2Scale 11.377777777 // 4096/360

//Robot_State
enum
{
    Initialization,
    Find_Ball,
    Trace_Ball,
    Goto_Ball,
    Find_Target,
    Move_to_Basket,
    Trace_Target,
    Goto_Target,
    UP_Basket,
	SlamDunk_Basket,
    End
};

//RobotPosition
enum
{
    BigGOAhead,
    TurnLeft,
    TurnRight
};

//ContinuousStep
enum
{
    ContinuousStand,
    ContinuousStay,
	ContinuousTurnRight,
    ContinuousSmallTurnRight,
	ContinuousTurnLeft,
    ContinuousSmallTurnLeft,
    ContinuousForward,
    ContinuousSmallForward,
    ContinuousSmallLeft,
    ContinuousBigLeft,
    ContinuousSmallRight,
    ContinuousBigRight,
    ContinuousFastForward,
    ContinuousFastSmallLeft,
    ContinuousFastBigLeft,
    ContinuousFastSmallRight,
    ContinuousFastBigRight,
    ContinuousBackward,
    ContinuousBackStart
};

class basketballinfo
{
public:

    void Init();

    struct Objectdata2
    {
        int XMax, XMin, YMax, YMin;
        int X,x;
        int Y,y;
        int Width;
        int Height;
        int size;
        int count;
    };
    Objectdata2 Ball;
    Objectdata2 Basket;

    union ContinuousStep_
    {
        struct
        {
            int InitX, InitY, InitZ, InitTheta;
            int Mode, IMUSet;
        }ContinuousInit;
        struct
        {
            int ExpX, ExpY, ExpZ, ExpTheta;
            int AddX, AddY, AddZ, AddTheta;
            int ChangeMode;
        }ContinuousMove;
    };
    double AddPeriod;
    ContinuousStep_ ContinuousStep[ContinuousMode];

    //loadparameter.cpp
    //29.ini calculate RobotHeight
    int M12;
    int M13;
    int M14;
    int M18;
    int M19;
    int M20;
    double RobotHeight;
    double RobotHeight1;
    double RobotHeight2;
    double RobotHeight3;
    double RobotStandFeedBack;
    double theta1;
    double theta2;
    double theta3;
    
    //strategy_main.cpp
    //strategymain
    bool LoadFlag;
    bool PrintFlag;

    //InversePerspective
    double AAngle;
    double BAngle;
    double BasketAngle;
    double dyAngle;
    double MiddleAngle;
    double RobotSearchBasketHeight;
    double ScreenButtom;
    double VisionMiddle;
    double Error;

    //Triangulation
    double DistanceError;   
    double DistanceErrorCount;
    double FeedBackError;
    double HeadVerticalAngle;

    //image
    int Ballcolor;
    int Basketcolor;

    //ComputeSpeed
    int dis35speed;
    int dis40speed;
    int dis50speed;
    int dis60speed;
    int dis61speed;
    int dis70speed;
    int dis71speed;
    int dis80speed;
    int dis81speed;
    int dis90speed;
    int disspeed;

    double dis35_x;
    double dis40_x;
    double dis50_x;
    double dis60_x;
    double dis65_x;
    double dis70_x;
    double dis75_x;
    double dis80_x;
    double dis85_x;
    double dis90_x;

    double weight_35;
    double weight_40;
    double weight_50;
    double weight_60;
    double weight_61;
    double weight_70;
    double weight_71;
    double weight_80;
    double weight_81;
    double weight_90;

    //SelectBaseLine
    int BasketVerticalBaseLine;
    int BasketVerticalBaseLine90;
    int BasketVerticalBaseLine80;
    int BasketVerticalBaseLine70;
    int BasketVerticalBaseLine60;
    int BasketVerticalBaseLine50;

    //TraceballHead
    bool PreRotateFlag;
    bool RestartFindBallFlag;
    bool StraightCatchFlag;
    double BallMoveX;
    double BallMoveY;
    double BasketMoveX;
    double BasketMoveY;
    double ImgHorizontalAngle;
    double ImgVerticalAngle;
    int backLine;

    //TraceballBody
    bool ContinuousFlag;
    bool StoopFlag;
    bool CatchBallModeFlag;
    bool MoveFlag;
    bool OutReturnFlag;
    bool InReturnFlag;
    bool GetBallFlag;
    bool TurnFlag;
    bool FaceBasketFlag;
    int CatchBallVerticalHeadPosition;
    int CatchBallYLine;
    int HandMove;
    double count;


    //TracebasketHead
    bool LayUpFlag;

    //Tracebasketbody
    bool RoateFlag;
    bool WaistFlag;
    bool ComputeFlag;
    bool ReAimFlag;
    bool RaiseFlag;
    bool ThrowBallFlag;
    int WaistError;

    //UPbasket
    bool BackFlag;
    bool LeftHandUpFlag;
    int UpBasketStopLine;

    //SlamDunk
    bool HandUpFlag;
    bool SlamDunkFlag;
    int SlamDunkHorizontalAngle;

    //Variables of Head
    int BallHorizontalBaseLine;
    int BallVerticalBaseLine;
    int BasketHorizontalBaseLine;
    int BasketHorizontalMaxAngle;
    int BasketHorizontalMinAngle;
    int HeadVerticalState;
    int HeadHorizontalState;
    int HeadTurnSpeed;
    int HorizontalHeadPosition;
    int HorizontalMaxAngle;
    int HorizontalMinAngle;
    int VerticalHeadPosition;
    int VerticalMaxAngle;
    int VerticalMinAngle;
    double ErrorHorizontalAngle;
    double ErrorVerticalAngle;

    //Common
    int BallHorizontalError;
    int BallVerticalError;
    int CatchBallLine;
    int ContinuousSlowLine;
    int ContinuousSlowLine2;
    int RobotPosition;
    int Robot_State;
    int SizeOfDist[4];
    double Distancenew;
    double timeuse;
};
extern basketballinfo *BasketInfo;

#endif
