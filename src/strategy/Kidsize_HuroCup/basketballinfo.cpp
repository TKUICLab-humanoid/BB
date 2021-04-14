#include "strategy/basketballinfo.h"

basketballinfo *BasketInfo = new basketballinfo();

void basketballinfo::Init()
{
    //loadparameter.cpp
    //29.ini calculate RobotHeight
    BasketInfo->M12 = 0;
    BasketInfo->M13 = 0;
    BasketInfo->M14 = 0;
    BasketInfo->M18 = 0;
    BasketInfo->M19 = 0;
    BasketInfo->M20 = 0;
    BasketInfo->RobotHeight = 0.0;
    BasketInfo->RobotHeight1 = 0.0;
    BasketInfo->RobotHeight2 = 0.0;
    BasketInfo->RobotHeight3 = 0.0;
    BasketInfo->RobotStandFeedBack = 0.0;
    BasketInfo->theta1 = 0.0;
    BasketInfo->theta2 = 0.0;
    BasketInfo->theta3 = 0.0;

    //strategy_main.cpp
    //main
    BasketInfo->LoadFlag = false;
    BasketInfo->PrintFlag = false;

    //InversePerspective
    BasketInfo->AAngle = 0.0;
    BasketInfo->BAngle = 0.0;
    BasketInfo->BasketAngle = 0.0;
    BasketInfo->dyAngle = 0.0;
    BasketInfo->MiddleAngle = 0.0;

    //Triangulation
    BasketInfo->DistanceError = 0.0;
    BasketInfo->HeadVerticalAngle = 0.0;

    //image
    BasketInfo->Ballcolor = 0;  //etOrange
    BasketInfo->Basketcolor = 5;//etRed

    //ComputeSpeed
    BasketInfo->disspeed = 0;
    BasketInfo->weight_35 = 0.0;
    BasketInfo->weight_40 = 0.0;
    BasketInfo->weight_50 = 0.0;
    BasketInfo->weight_60 = 0.0;
    BasketInfo->weight_61 = 0.0;
    BasketInfo->weight_70 = 0.0;
    BasketInfo->weight_71 = 0.0;
    BasketInfo->weight_80 = 0.0;
    BasketInfo->weight_81 = 0.0;
    BasketInfo->weight_90 = 0.0;

    //TraceballHead
    BasketInfo->PreRotateFlag = false;
    BasketInfo->RestartFindBallFlag = true;
    BasketInfo->StraightCatchFlag = true;
    BasketInfo->BallMoveX = 0.0;
    BasketInfo->BallMoveY = 0.0;
    BasketInfo->BasketMoveX = 0.0;
    BasketInfo->BasketMoveY = 0.0;
    BasketInfo->ImgHorizontalAngle = 70.42;
    BasketInfo->ImgVerticalAngle = 43.3;

    //TraceballBody
    BasketInfo->ContinuousFlag = true;
    BasketInfo->StoopFlag = false;
    BasketInfo->CatchBallModeFlag = false;
    BasketInfo->MoveFlag = false;
    BasketInfo->OutReturnFlag = false;
    BasketInfo->InReturnFlag = false;
    BasketInfo->GetBallFlag = false;
    BasketInfo->TurnFlag = false;
    BasketInfo->FaceBasketFlag = false;
    BasketInfo->HandMove = 0;
    BasketInfo->count = 0.0;

    //TracebasketHead
    BasketInfo->LayUpFlag = false;

    //Tracebasketbody
    BasketInfo->RoateFlag = true;
    BasketInfo->WaistFlag = false;
    BasketInfo->ComputeFlag = false;
    BasketInfo->ReAimFlag = false;
    BasketInfo->RaiseFlag = false;
    BasketInfo->ThrowBallFlag = false;

    //UPbasket
    BasketInfo->LeftHandUpFlag = false;

    //SlamDunk
    BasketInfo->HandUpFlag = true;
    BasketInfo->SlamDunkFlag = false;


    //Variables of Head
    BasketInfo->HeadTurnSpeed = 40;
    BasketInfo->HorizontalHeadPosition = 0;
    BasketInfo->VerticalHeadPosition = 0;
    BasketInfo->ErrorHorizontalAngle = 0.0;
    BasketInfo->ErrorVerticalAngle = 0.0;

 	//Common
 	BasketInfo->RobotPosition = BigGOAhead;
 	BasketInfo->Robot_State = Initialization;
    BasketInfo->Distancenew = 0.0;
    BasketInfo->timeuse = 0;
}
