#include "strategy/loadparameter.h"

LoadParameter *Load = new LoadParameter();

float LoadParameter::readvalue(fstream &fin, string title, int mode)
{
    char line[100]; 
    char equal;
    switch(mode)
    {
        case 0:
            while(1)
            {
                fin.getline(line,sizeof(line),' ');
                if((string)line == title)
                {
                    fin.get(equal);
                    if(equal == '=')
                    {
                        fin.getline(line,sizeof(line),'\n');
                        break;
                    }
                }
            }
            return atoi(line);
            break;
        case 1:
            while(1)
            {
                fin.getline(line,sizeof(line),' ');
                if((string)line == title)
                {
                    fin.get(equal);
                    if(equal == '=')
                    {
                        fin.getline(line,sizeof(line),'\n');
                        break;
                    }
                }
            }
            return atof(line);
            break;
        case 2:
            while(1)
            {
                fin.getline(line,sizeof(line),' ');
                if((string)line == title)
                {
                    fin.get(equal);
                    if(equal == '=')
                    {
                        fin.getline(line,sizeof(line),'|');
                        break;
                    }
                }
            }
            return atoi(line);
            break;
        case 3:
            while(1)
            {
                fin.getline(line,sizeof(line),' ');
                if((string)line == title)
                {
                    fin.getline(line,sizeof(line),'|');
                    break;
                }
            }
            return atoi(line);
            break;
        default:
            break;
    }
}
void LoadParameter::initparameterpath()
{
	while(parameter_path == "N")
	{
		parameter_path = tool->getPackagePath("strategy");
	}
	printf("parameter_path is %s\n", parameter_path.c_str());
}
void LoadParameter::LoadParameters()
{
    //0->int        1->float  
    fstream fin;
    string sTmp;
    char line[100];
    char path_stand[200];
    char path_stand2[20] = "/sector/";
    char path_stand3[20] = "29.ini";
    char path_throw[200],path_speed[200],path_continuous[200];
    char temp[100];
    int packagecnt;
    vector<int> StandPackage;
    strcpy(path_stand, tool->standPath);
    strcat(path_stand, path_stand2);
    strcat(path_stand, path_stand3);
    fin.open(path_stand, ios::in);
    try//計算機器人的身高
    {
        ROS_INFO("openfile Stand");
        packagecnt = tool->readvalue(fin, "PackageCnt", 0);
        StandPackage.push_back(tool->readvalue(fin, "Package", 2));
        for(int i = 1; i < packagecnt; i++)
        {
            StandPackage.push_back(tool->readvalue(fin, "|", 3));
        }
        BasketInfo->M12 = 2048 - (StandPackage[47] + StandPackage[48] * 256);
        BasketInfo->M13 = 2048 - (StandPackage[51] + StandPackage[52] * 256);
        BasketInfo->M14 = 2048 - (StandPackage[55] + StandPackage[56] * 256);
        BasketInfo->M18 = 2048 - (StandPackage[71] + StandPackage[72] * 256);
        BasketInfo->M19 = 2048 - (StandPackage[75] + StandPackage[76] * 256);
        BasketInfo->M20 = 2048 - (StandPackage[79] + StandPackage[80] * 256);
       
        BasketInfo->theta1 = (abs(BasketInfo->M14) + abs(BasketInfo->M20)) / 2 * Scale2Deg;
        BasketInfo->theta2 = (abs(BasketInfo->M13) + abs(BasketInfo->M19)) / 2 * Scale2Deg;
        BasketInfo->theta3 = (abs(BasketInfo->M12) + abs(BasketInfo->M18)) / 2 * Scale2Deg;

        BasketInfo->RobotHeight1 = cos(BasketInfo->theta1 * Deg2Rad) * L_Calf; 
        BasketInfo->RobotHeight2 = cos((BasketInfo->theta2 - BasketInfo->theta1) * Deg2Rad) * L_Thigh; 
        BasketInfo->RobotHeight3 = cos((BasketInfo->theta3 + L_BodyError - BasketInfo->theta2 + BasketInfo->theta1) * Deg2Rad) * L_Body; 

        BasketInfo->RobotStandFeedBack = (BasketInfo->theta2 - BasketInfo->theta1 - BasketInfo->theta3);
        BasketInfo->RobotHeight = BasketInfo->RobotHeight1 + BasketInfo->RobotHeight2 + BasketInfo->RobotHeight3 + L_Foot + L_Shoes - HalfBasketHeight;
        fin.close();
        ROS_INFO("fileclose Stand");
    }
    catch(exception e)
    {
        ROS_INFO("catchfile");
    }
    StandPackage.clear();

    strcpy(path_throw, parameter_path.c_str());
    strcat(path_throw, "/Throw.ini");
    fin.open(path_throw, ios::in);
    //Throw.ini
    try//策略中的各項參數
    {
        ROS_INFO("openfile Throw.ini");
        fin.getline(temp, sizeof(temp));
        BasketInfo->RobotSearchBasketHeight = this->readvalue(fin,"RobotSearchBasketHeight", 1);
        BasketInfo->VisionMiddle =  this->readvalue(fin,"VisionMiddle", 1);
        BasketInfo->ScreenButtom = this->readvalue(fin,"ScreenButtom", 1);
        BasketInfo->Error = this->readvalue(fin,"Error", 1);
        fin.getline(temp, sizeof(temp));
        BasketInfo->FeedBackError = this->readvalue(fin,"FeedBackError", 1);
        BasketInfo->DistanceErrorCount = this->readvalue(fin,"DistanceErrorCount", 1);
        fin.getline(temp, sizeof(temp));
        BasketInfo->HeadVerticalState = this->readvalue(fin,"HeadVerticalState", 0);
        BasketInfo->HeadHorizontalState = this->readvalue(fin,"HeadHorizontalState", 0);
        BasketInfo->BallVerticalBaseLine = this->readvalue(fin,"BallVerticalBaseLine", 0);
        BasketInfo->BallHorizontalBaseLine = this->readvalue(fin,"BallHorizontalBaseLine", 0);
        BasketInfo->VerticalMaxAngle = this->readvalue(fin,"VerticalMaxAngle", 0);
        BasketInfo->VerticalMinAngle = this->readvalue(fin,"VerticalMinAngle", 0);
        BasketInfo->HorizontalMaxAngle = this->readvalue(fin,"HorizontalMaxAngle", 0);
        BasketInfo->HorizontalMinAngle = this->readvalue(fin,"HorizontalMinAngle", 0);
        BasketInfo->BasketHorizontalMaxAngle = this->readvalue(fin,"BasketHorizontalMaxAngle", 0);
        BasketInfo->BasketHorizontalMinAngle = this->readvalue(fin,"BasketHorizontalMinAngle", 0);
        BasketInfo->BasketHorizontalBaseLine = this->readvalue(fin,"BasketHorizontalBaseLine", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->BasketVerticalBaseLine = this->readvalue(fin,"BasketVerticalBaseLine", 0);
        BasketInfo->BasketVerticalBaseLine90 = this->readvalue(fin,"BasketVerticalBaseLine90", 0);
        BasketInfo->BasketVerticalBaseLine80 = this->readvalue(fin,"BasketVerticalBaseLine80", 0);
        BasketInfo->BasketVerticalBaseLine70 = this->readvalue(fin,"BasketVerticalBaseLine70", 0);
        BasketInfo->BasketVerticalBaseLine60 = this->readvalue(fin,"BasketVerticalBaseLine60", 0);
	    BasketInfo->BasketVerticalBaseLine50 = this->readvalue(fin,"BasketVerticalBaseLine50", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->SizeOfDist[0] = this->readvalue(fin,"Size50", 0);
        BasketInfo->SizeOfDist[1] = this->readvalue(fin,"Size60", 0);
        BasketInfo->SizeOfDist[2] = this->readvalue(fin,"Size70", 0);
        BasketInfo->SizeOfDist[3] = this->readvalue(fin,"Size80", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->BallVerticalError = this->readvalue(fin,"BallVerticalError", 0);
        BasketInfo->BallHorizontalError = this->readvalue(fin,"BallHorizontalError", 0);
        BasketInfo->ContinuousSlowLine = this->readvalue(fin,"ContinuousSlowLine", 0);
        BasketInfo->CatchBallLine = this->readvalue(fin,"CatchBallLine", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->CatchBallVerticalHeadPosition = this->readvalue(fin,"CatchBallVerticalHeadPosition", 0);
        BasketInfo->CatchBallYLine = this->readvalue(fin,"CatchBallYLine", 0);
        BasketInfo->backLine = this->readvalue(fin,"backLine", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->WaistError = this->readvalue(fin,"WaistError", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->UpBasketStopLine = this->readvalue(fin,"UpBasketStopLine", 0);
        BasketInfo->ContinuousSlowLine2 = this->readvalue(fin,"ContinuousSlowLine2", 0);
        BasketInfo->SlamDunkHorizontalAngle = this->readvalue(fin,"SlamDunkHorizontalAngle", 0);
        fin.close();
        ROS_INFO("fileclose Throw.ini");
    }
    catch(exception e)
    {
        ROS_INFO("catchfile");
    }
    
    strcpy(path_speed, parameter_path.c_str());
    strcat(path_speed, "/Speed.ini");
    fin.open(path_speed, ios::in);
    //speed.ini
    try//投籃力道參數
    {
        ROS_INFO("openfile Speed.ini");
        BasketInfo->dis35_x = this->readvalue(fin, "dis35_x", 1);
        BasketInfo->dis40_x = this->readvalue(fin, "dis40_x", 1);
        BasketInfo->dis50_x = this->readvalue(fin, "dis50_x", 1);
        BasketInfo->dis60_x = this->readvalue(fin, "dis60_x", 1);
        BasketInfo->dis65_x = this->readvalue(fin, "dis65_x", 1);
        BasketInfo->dis70_x = this->readvalue(fin, "dis70_x", 1);
        BasketInfo->dis75_x = this->readvalue(fin, "dis75_x", 1);
        BasketInfo->dis80_x = this->readvalue(fin, "dis80_x", 1);
        BasketInfo->dis85_x = this->readvalue(fin, "dis85_x", 1);
        BasketInfo->dis90_x = this->readvalue(fin, "dis90_x", 1);
        BasketInfo->dis35speed = this->readvalue(fin, "dis35speed", 0);
        BasketInfo->dis40speed = this->readvalue(fin, "dis40speed", 0);
        BasketInfo->dis50speed = this->readvalue(fin, "dis50speed", 0);
        BasketInfo->dis60speed = this->readvalue(fin, "dis60speed", 0);
        BasketInfo->dis61speed = this->readvalue(fin, "dis61speed", 0);
        BasketInfo->dis70speed = this->readvalue(fin, "dis70speed", 0);
        BasketInfo->dis71speed = this->readvalue(fin, "dis71speed", 0);
        BasketInfo->dis80speed = this->readvalue(fin, "dis80speed", 0);
        BasketInfo->dis81speed = this->readvalue(fin, "dis81speed", 0);
        BasketInfo->dis90speed = this->readvalue(fin, "dis90speed", 0);
       
        fin.close();
        ROS_INFO("fileclose Speed.ini");
    }
    catch(exception e)
    {
        ROS_INFO("catchfile");
    }
    
    strcpy(path_continuous, parameter_path.c_str());
    strcat(path_continuous, "/ContinuousMove.ini");
    fin.open(path_continuous, ios::in);
    //ContinuousMove.ini
    try//連續步態參數
    {
        ROS_INFO("openfile ContinuousMove.ini");

        BasketInfo->AddPeriod = this->readvalue(fin, "AddPeriod", 1);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitX = this->readvalue(fin, "ContinuousStep_InitX", 0);
        BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitY = this->readvalue(fin, "ContinuousStep_InitY", 0);
        BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitZ = this->readvalue(fin, "ContinuousStep_InitZ", 0);
        BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitTheta = this->readvalue(fin, "ContinuousStep_InitTheta", 0);
        BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode = this->readvalue(fin, "ContinuousStep_Mode", 0);
        BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.IMUSet = this->readvalue(fin, "ContinuousStep_IMU", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_StayExpX", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_StayExpY", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_StayExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_StayExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_StayAddX", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_StayAddY", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_StayAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_StayAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_StayChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_TurnRightExpX", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_TurnRightExpY", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_TurnRightExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_TurnRightExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_TurnRightAddX", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_TurnRightAddY", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_TurnRightAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_TurnRightAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousTurnRight].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_TurnRightChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_SmallTurnRightExpX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_SmallTurnRightExpY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_SmallTurnRightExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_SmallTurnRightExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_SmallTurnRightAddX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_SmallTurnRightAddY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_SmallTurnRightAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_SmallTurnRightAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnRight].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_SmallTurnRightChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_TurnLeftExpX", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_TurnLeftExpY", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_TurnLeftExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_TurnLeftExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_TurnLeftAddX", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_TurnLeftAddY", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_TurnLeftAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_TurnLeftAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousTurnLeft].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_TurnLeftChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_SmallTurnLeftExpX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_SmallTurnLeftExpY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_SmallTurnLeftExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_SmallTurnLeftExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_SmallTurnLeftAddX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_SmallTurnLeftAddY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_SmallTurnLeftAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_SmallTurnLeftAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallTurnLeft].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_SmallTurnLeftChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_ForwardExpX", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_ForwardExpY", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_ForwardExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_ForwardExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_ForwardAddX", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_ForwardAddY", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_ForwardAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_ForwardAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousForward].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_ForwardChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_SmallForwardExpX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_SmallForwardExpY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_SmallForwardExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_SmallForwardExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_SmallForwardAddX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_SmallForwardAddY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_SmallForwardAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_SmallForwardAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallForward].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_SmallForwardChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_SmallLeftExpX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_SmallLeftExpY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_SmallLeftExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_SmallLeftExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_SmallLeftAddX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_SmallLeftAddY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_SmallLeftAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_SmallLeftAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallLeft].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_SmallLeftChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_BigLeftExpX", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_BigLeftExpY", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_BigLeftExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_BigLeftExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_BigLeftAddX", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_BigLeftAddY", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_BigLeftAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_BigLeftAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBigLeft].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_BigLeftChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_SmallRightExpX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_SmallRightExpY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_SmallRightExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_SmallRightExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_SmallRightAddX", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_SmallRightAddY", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_SmallRightAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_SmallRightAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousSmallRight].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_SmallRightChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_BigRightExpX", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_BigRightExpY", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_BigRightExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_BigRightExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_BigRightAddX", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_BigRightAddY", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_BigRightAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_BigRightAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBigRight].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_BigRightChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_FastForwardExpX", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_FastForwardExpY", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_FastForwardExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_FastForwardExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_FastForwardAddX", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_FastForwardAddY", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_FastForwardAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_FastForwardAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastForward].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_FastForwardChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_FastSmallLeftExpX", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_FastSmallLeftExpY", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_FastSmallLeftExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_FastSmallLeftExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_FastSmallLeftAddX", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_FastSmallLeftAddY", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_FastSmallLeftAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_FastSmallLeftAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallLeft].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_FastSmallLeftChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_FastBigLeftExpX", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_FastBigLeftExpY", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_FastBigLeftExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_FastBigLeftExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_FastBigLeftAddX", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_FastBigLeftAddY", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_FastBigLeftAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_FastBigLeftAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigLeft].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_FastBigLeftChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_FastSmallRightExpX", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_FastSmallRightExpY", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_FastSmallRightExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_FastSmallRightExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_FastSmallRightAddX", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_FastSmallRightAddY", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_FastSmallRightAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_FastSmallRightAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastSmallRight].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_FastSmallRightChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_FastBigRightExpX", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_FastBigRightExpY", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_FastBigRightExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_FastBigRightExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_FastBigRightAddX", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_FastBigRightAddY", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_FastBigRightAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_FastBigRightAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousFastBigRight].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_FastBigRightChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.ExpX = this->readvalue(fin, "ContinuousStep_BackwardExpX", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.ExpY = this->readvalue(fin, "ContinuousStep_BackwardExpY", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.ExpZ = this->readvalue(fin, "ContinuousStep_BackwardExpZ", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.ExpTheta = this->readvalue(fin, "ContinuousStep_BackwardExpTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.AddX = this->readvalue(fin, "ContinuousStep_BackwardAddX", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.AddY = this->readvalue(fin, "ContinuousStep_BackwardAddY", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.AddZ = this->readvalue(fin, "ContinuousStep_BackwardAddZ", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.AddTheta = this->readvalue(fin, "ContinuousStep_BackwardAddTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBackward].ContinuousMove.ChangeMode = this->readvalue(fin, "ContinuousStep_BackwardChangeMode", 0);
        fin.getline(temp, sizeof(temp));
        BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitX = this->readvalue(fin, "ContinuousStep_BackInitX", 0);
        BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitY = this->readvalue(fin, "ContinuousStep_BackInitY", 0);
        BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitZ = this->readvalue(fin, "ContinuousStep_BackInitZ", 0);
        BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitTheta = this->readvalue(fin, "ContinuousStep_BackInitTheta", 0);
        BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.Mode = this->readvalue(fin, "ContinuousStep_BackMode", 0);
        BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.IMUSet = this->readvalue(fin, "ContinuousStep_BackIMU", 0);

        fin.close();
        ROS_INFO("fileclose ContinuousMove.ini");
    }
    catch(exception e)
    {
        ROS_INFO("catchfile");
    }
}

void LoadParameter::TestParameters()
{
    ROS_INFO("---------------  Throw ------------------");
    ROS_INFO("RobotSearchBasketHeight = %f",BasketInfo->RobotSearchBasketHeight);
    ROS_INFO("VisionMiddle = %f",BasketInfo->VisionMiddle);
    ROS_INFO("ScreenButtom = %f",BasketInfo->ScreenButtom);
    ROS_INFO("Error = %f",BasketInfo->Error);
    ROS_INFO("HeadVerticalState = %d",BasketInfo->HeadVerticalState);
    ROS_INFO("HeadHorizontalState = %d",BasketInfo->HeadHorizontalState);
    ROS_INFO("BallVerticalBaseLine = %d",BasketInfo->BallVerticalBaseLine);
    ROS_INFO("BallHorizontalBaseLine = %d",BasketInfo->BallHorizontalBaseLine);
    ROS_INFO("VerticalMaxAngle = %d",BasketInfo->VerticalMaxAngle);
    ROS_INFO("VerticalMinAngle = %d",BasketInfo->VerticalMinAngle);
    ROS_INFO("HorizontalMaxAngle = %d",BasketInfo->HorizontalMaxAngle);
    ROS_INFO("HorizontalMinAngle = %d",BasketInfo->HorizontalMinAngle);
    ROS_INFO("BasketHorizontalMinAngle = %d",BasketInfo->BasketHorizontalMinAngle);
    ROS_INFO("BasketHorizontalMaxAngle = %d",BasketInfo->BasketHorizontalMaxAngle);
    ROS_INFO("BasketHorizontalBaseLine = %d",BasketInfo->BasketHorizontalBaseLine);
    ROS_INFO("BasketVerticalBaseLine = %d",BasketInfo->BasketVerticalBaseLine);
    ROS_INFO("BasketVerticalBaseLine90 = %d",BasketInfo->BasketVerticalBaseLine90);
    ROS_INFO("BasketVerticalBaseLine80 = %d",BasketInfo->BasketVerticalBaseLine80);
    ROS_INFO("BasketVerticalBaseLine70 = %d",BasketInfo->BasketVerticalBaseLine70);
    ROS_INFO("BasketVerticalBaseLine60 = %d",BasketInfo->BasketVerticalBaseLine60);
    ROS_INFO("BasketVerticalBaseLine50 = %d",BasketInfo->BasketVerticalBaseLine50);
    ROS_INFO("WaistError = %d",BasketInfo->WaistError);
    ROS_INFO("BallVerticalError = %d",BasketInfo->BallVerticalError);
    ROS_INFO("BallHorizontalError = %d",BasketInfo->BallHorizontalError);
    ROS_INFO("ContinuousSlowLine = %d",BasketInfo->ContinuousSlowLine);
    ROS_INFO("ContinuousSlowLine2 = %d",BasketInfo->ContinuousSlowLine2);
    ROS_INFO("CatchBallLine = %d",BasketInfo->CatchBallLine);
    ROS_INFO("---------------  Speed ------------------");
    ROS_INFO("dis35_x = %f",BasketInfo->dis35_x);
    ROS_INFO("dis40_x = %f",BasketInfo->dis40_x);
    ROS_INFO("dis50_x = %f",BasketInfo->dis50_x);
    ROS_INFO("dis60_x = %f",BasketInfo->dis60_x);
    ROS_INFO("dis65_x = %f",BasketInfo->dis65_x);
    ROS_INFO("dis70_x = %f",BasketInfo->dis70_x);
    ROS_INFO("dis75_x = %f",BasketInfo->dis75_x);
    ROS_INFO("dis80_x = %f",BasketInfo->dis80_x);
    ROS_INFO("dis85_x = %f",BasketInfo->dis85_x);
    ROS_INFO("dis90_x = %f",BasketInfo->dis90_x);
    ROS_INFO("dis35speed = %d",BasketInfo->dis35speed);
    ROS_INFO("dis40speed = %d",BasketInfo->dis40speed);
    ROS_INFO("dis50speed = %d",BasketInfo->dis50speed);
    ROS_INFO("dis60speed = %d",BasketInfo->dis60speed);
    ROS_INFO("dis61speed = %d",BasketInfo->dis61speed);
    ROS_INFO("dis70speed = %d",BasketInfo->dis70speed);
    ROS_INFO("dis71speed = %d",BasketInfo->dis71speed);
    ROS_INFO("dis80speed = %d",BasketInfo->dis80speed);
    ROS_INFO("dis81speed = %d",BasketInfo->dis81speed);
    ROS_INFO("dis90speed = %d",BasketInfo->dis90speed);
    
    ROS_INFO("----------  Continuous Step -------------");
    ROS_INFO("AddPeriod = %f",BasketInfo->AddPeriod);
    ROS_INFO("ContinuousInit");
    ROS_INFO("InitX = %d",BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitX);
    ROS_INFO("InitY = %d",BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitY);
    ROS_INFO("InitZ = %d",BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitZ);
    ROS_INFO("InitTheta = %d",BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.InitTheta);
    ROS_INFO("Mode = %d",BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode);
    ROS_INFO("IMUSet = %d",BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.IMUSet);
    ROS_INFO("etContinuousBackInit");
    ROS_INFO("InitX = %d",BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitX);
    ROS_INFO("InitY = %d",BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitY);
    ROS_INFO("InitZ = %d",BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitZ);
    ROS_INFO("InitTheta = %d",BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.InitTheta);
    ROS_INFO("Mode = %d",BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.Mode);
    ROS_INFO("IMUSet = %d",BasketInfo->ContinuousStep[ContinuousBackStart].ContinuousInit.IMUSet);

    for(int i = ContinuousStand; i <= ContinuousBackStart; i++)
    {
        switch(i)
        {
            case ContinuousStand:
                ROS_INFO("ContinuousStand");
                break;
            case ContinuousStay:
                ROS_INFO("ContinuousStay");
                break;
            case ContinuousTurnRight:
                ROS_INFO("ContinuousTurnRight");
                break;
            case ContinuousSmallTurnRight:
                ROS_INFO("ContinuousSmallTurnRight");
                break;
            case ContinuousTurnLeft:
                ROS_INFO("ContinuousTurnLeft");
                break;
            case ContinuousSmallTurnLeft:
                ROS_INFO("ContinuousSmallTurnLeft");
                break;
            case ContinuousForward:
                ROS_INFO("ContinuousForward");
                break;
            case ContinuousSmallForward:
                ROS_INFO("ContinuousSmallForward");
                break;
            case ContinuousSmallLeft:
                ROS_INFO("ContinuousSmallLeft");
                break;
            case ContinuousBigLeft:
                ROS_INFO("ContinuousBigLeft");
                break;
            case ContinuousSmallRight:
                ROS_INFO("ContinuousSmallRight");
                break;
            case ContinuousBigRight:
                ROS_INFO("ContinuousBigRight");
                break;
            case ContinuousFastForward:
                ROS_INFO("ContinuousFastForward");
                break;
            case ContinuousFastSmallLeft:
                ROS_INFO("ContinuousFastSmallLeft");
                break;
            case ContinuousFastBigLeft:
                ROS_INFO("ContinuousFastBigLeft");
                break;
            case ContinuousFastSmallRight:
                ROS_INFO("ContinuousFastSmallRight");
                break;
            case ContinuousFastBigRight:
                ROS_INFO("ContinuousFastBigRight");
                break;
            case ContinuousBackward:
                ROS_INFO("ContinuousBackward");
                break;
            case ContinuousBackStart:
                ROS_INFO("ContinuousBackStart");
                break;
        }
        ROS_INFO("ExpX = %d",BasketInfo->ContinuousStep[i].ContinuousMove.ExpX);
        ROS_INFO("ExpY = %d",BasketInfo->ContinuousStep[i].ContinuousMove.ExpY);
        ROS_INFO("ExpZ = %d",BasketInfo->ContinuousStep[i].ContinuousMove.ExpZ);
        ROS_INFO("ExpTheta = %d",BasketInfo->ContinuousStep[i].ContinuousMove.ExpTheta);
        ROS_INFO("AddX = %d",BasketInfo->ContinuousStep[i].ContinuousMove.AddX);
        ROS_INFO("AddY = %d",BasketInfo->ContinuousStep[i].ContinuousMove.AddY);
        ROS_INFO("AddZ = %d",BasketInfo->ContinuousStep[i].ContinuousMove.AddZ);
        ROS_INFO("AddTheta = %d",BasketInfo->ContinuousStep[i].ContinuousMove.AddTheta);
    }
}
