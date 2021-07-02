#include "strategy/strategy_main.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "BBthrow");
	ros::NodeHandle nh;
	KidsizeStrategy KidsizeStrategy;
    
	ros::Rate loop_rate(30);

    Load->initparameterpath();

	while (nh.ok()) 
	{
		ros::spinOnce();
		KidsizeStrategy.strategymain();
		loop_rate.sleep();
	}
	return 0;
}

void KidsizeStrategy::strategymain()
{
	if(strategy_info->getStrategyStart())//策略指撥開關
	{	
		if(!DIOSTRATAGAIN)
		{
		    BasketInfo->Init();
		    Load->LoadParameters();
		    FindballInitial();
		    DIOSTRATAGAIN = true;
		    sendbodystandflag = false;
		}		
		image();
		Draw();

		switch(BasketInfo->Robot_State)
		{
		    case Initialization:
		        ROS_INFO("Initialize");
                if(strategy_info->DIOValue.Switch.D0)//1號小指撥開啟上籃策略
                {
                    BasketInfo->LayUpFlag = true;
                }
                else if(strategy_info->DIOValue.Switch.D2)
                {
                    BasketInfo->FivePointFlag = true;
                }
		        MoveContinuous(ContinuousStand);//設定步態初始化參數
		    	tool->Delay(1000);
                ros_com->sendBodySector(BB_StandFix);
                tool->Delay(1000);
                ros_com->sendSensorReset();//IMU值重製
		        BasketInfo->Robot_State = Find_Ball;
		        break;
		    case Find_Ball:
		        FindballHead();
		        break;
		    case Trace_Ball:
		        TraceballHead();			
		        break;
		    case Goto_Ball:
		        TraceballBody();
		        break;
		    case Find_Target:
		        FindbasketHead();
		        break;
		    case Trace_Target:
		        TracebasketHead();
		        break;
		    case Goto_Target:
		        TracebasketBody();
		        break;
		    case UP_Basket:
				UPbasket();
				break;
			case SlamDunk_Basket:
				SlamDunk();
				break;
		    case End:
		        break;
		} 
	}
	else
	{
		if(walk_con->isStartContinuous())//判斷是否開啟連續步態
        {
            walk_con->stopContinuous();//關閉連續步態
            tool->Delay(1500);
        }
        if(!BasketInfo->LoadFlag)
        {
        	BasketInfo->Init();//參數初始化
            Load->LoadParameters();//讀檔
            BasketInfo->LoadFlag = true;
        }
        if(!sendbodystandflag)
        {
            ros_com->sendBodySector(Robot_StandUp);
            sendbodystandflag = true;
            DIOSTRATAGAIN = false;
        }
        if(strategy_info->DIOValue.Switch.D1)//2號小指撥開啟時顯示線，主要用於測試各項參數
        {            
            image();
            Draw();
            Triangulation();
	        ROS_INFO("----------------------------------------");
            ROS_INFO("BasketInfo->Basket.size = %d", BasketInfo->Basket.size);
            ROS_INFO("BasketInfo->Basket.YMax = %d", BasketInfo->Basket.YMax);
            ROS_INFO("BasketInfo->Distancenew = %f", BasketInfo->Distancenew);
            ROS_INFO("BasketInfo->Ball.Y = %d", BasketInfo->Ball.Y);
            ROS_INFO("IMU = %f", strategy_info->getIMUValue().Yaw);
            ROS_INFO("Basket.X = %d", BasketInfo->Basket.X);
	        ROS_INFO("Ball.size = %d", BasketInfo->Ball.size);
	        ROS_INFO("----------------------------------------");
        }
        if(!BasketInfo->PrintFlag)
        {      
            std::printf("        ▃ \n");
            std::printf("　　　 　▋ 　　 ▋　 ◢▀　▀◣ \n");
            std::printf("　　　　▌　 　 　▌　▌ 　 .▌ \n");
            std::printf("　　 　 ▌　　　　▌ ▌　　　▌ \n");
            std::printf("　 　　▐ 　 　　 ▌ ▌ 　 　▌ \n");
            std::printf("　 　　 ▐ 　 　 ▀■▀ 　 　▌ \n");
            std::printf("　　　◢◤　　　　　　　　　▀▃ \n");
            std::printf("　　◢◤　　　　　　　　　 　　◥◣ \n");
            std::printf("　　▌　　　　　　　　　　 　　　▌ \n");
            std::printf("　 ▐　 　●　　 　　　　●　　　　▌ 　 \n");
            std::printf("　　▌　　　　　　　　　　　　　 ▌ \n");
            std::printf("　　◥◣ 　 　　 ╳ 　　　　　　◢◤ \n");
            std::printf("　　 ◢▀▅▃▂　　　  ▂▂▃▅▀▅ ◢◤\n");
            std::printf("　◢◤　　　　▀▀▀▀▀　　　　　◥◣ \n");
            std::printf("▐◣▃▌　　　　　　　　　　　▐▃◢▌ \n");
            std::printf("◥◣▃▌　　　　 　　 　　　　▐▃◢◤ \n");
            std::printf("　 ▀▅▃　　　　　 　 　　▂▅▀ \n");
            std::printf("　　 　 ▀■▆▅▅▅▅▅▆■█▀ \n");
            std::printf("　　　 　▐▃▃▃▲▃▃▃◢ \n\n\n");
            std::printf("-----------------------------------\n");
            std::printf("        ▐ ▀◣     ◢▀ ◣\n");
            std::printf("       ◢　  ▌    ▌　  ▌\n");
            std::printf("        ▌　  ▌   ▀ 　  ▌\n");
            std::printf("         ▀     ▃ ▌ 　　 ▌▅▃▂▂▂▂▂\n");
            std::printf("          ▌                     ▀▌\n");
            std::printf("  　   　 ◢◤　　　　　　　　　    ▌\n");
            std::printf("  　   　◤　　　　　　   ● 　　　 ▐ \n");
            std::printf("        ▌　 　  ●　              ▂▐ \n");
            std::printf("       ▌　 　  　　 　　　　   ▂▂▌\n");
            std::printf("        ▌　 　  　　 W　　　  ▌ 　   \n");
            std::printf("  　   　◥◣ 　 　　    　　 ◢◤        \n");
            std::printf("    　    ◢▀▅▃▂　　　  ▂ ◢◤       ◢◤\n");
            std::printf(" ◥◣    ◢◤　　　　▀▀▀▀▀　  ◥◣▂   ◢◤\n");
            std::printf("   ◥◣ ▐◤                    ▐ ◢◤\n");
            std::printf("      ▌                      ▐\n");
            std::printf("      ▌　　　 　　 　　　　  ▐\n");
            std::printf("      ▐                      ▐\n"); 
            std::printf("        ▌  　　　　 　　 　　▌\n");
            std::printf("  　     ▀▀▅▃　　　　　 　▂▅▀\n");
            std::printf("  　　    　 ▀■▆▅▅▅▅▅▆■█▀ \n");
            std::printf("  	        ▌        ▌\n");  
            std::printf("                ██       ██ \n");
            std::printf("-----------------------------------\n");
            std::printf("　  　      ▃▀▀▀▀▀▀▀▀▀▃n");
            std::printf("　  　　◢◤　　　　　　    ▀▃\n");
            std::printf("  　　◢◤　　　　　　　　 　 ◥◣ \n");
            std::printf("  　　▌　　　 ●--------●    ▌ \n");
            std::printf("  　  ▌　 　  　　 　　　  　 ▌ 　 \n");
            std::printf("　  　◥◣ 　 　　  　　　   ◢◤ \n");
            std::printf("　　   ◢▀▅▃▂         ▂▃▅▀\n");
            std::printf("　   ◢◤　　　             ◥◣ \n");
            std::printf("   ◢  ▌  　　　　　　　 　　▐  ◣ \n");
            std::printf("  ▌  ▌　　　　 　　 　  　　▐   ◣\n");
            std::printf(" ▌  ▌　　　　　　　　　  　  ▐   ▌ \n");
            std::printf(" ▌  ▌　　　　 　　　　　   　 ▐   ▌ \n");
            std::printf("  ◣ ▌　　　　　　　　　　    ▐  ◤ \n");
            std::printf("   ◣▌　　　　　　 　　　 　  ▐◢◤ \n");
            std::printf("　 ▀▅▃　　　　 　 　　▂  ▅▀ \n");
            std::printf("　　 　  ▀■▆▅▅▅▅▅▆■█▀ \n");
            std::printf("　　　   ◣     ▲     ◢ \n\n\n");
            std::printf("　　　    ◣▃▃◢   ◣▃▃◢ \n\n\n");
            BasketInfo->PrintFlag = true;
            MoveHead(HeadMotorID::VerticalID, 2048, 200);//1623
            MoveHead(HeadMotorID::HorizontalID, 2048, 200);
        }
	}
}

void KidsizeStrategy::Draw()//顯示出籃框的線&&球模的線
{
    ros_com->drawImageFunction(1, DrawMode::DrawLine, 0, 320, 120, 120, 152, 245, 255);//用於逆透視法測距的VisionMiddle blue
    ros_com->drawImageFunction(2, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine50, BasketInfo->BasketVerticalBaseLine50, 0, 240, 255, 0, 0);//50 red
    ros_com->drawImageFunction(3, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine60, BasketInfo->BasketVerticalBaseLine60, 0, 240, 255, 0, 0);//60 red
    ros_com->drawImageFunction(4, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine70, BasketInfo->BasketVerticalBaseLine70, 0, 240, 255, 0, 0);//70 red
    ros_com->drawImageFunction(5, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine80, BasketInfo->BasketVerticalBaseLine80, 0, 240, 255, 0, 0);//80 red
    ros_com->drawImageFunction(6, DrawMode::DrawLine, BasketInfo->BasketVerticalBaseLine , BasketInfo->BasketVerticalBaseLine , 0, 240, 255, 0, 0);//default red
    ros_com->drawImageFunction(7, DrawMode::DrawObject, BasketInfo->Ball.XMin, BasketInfo->Ball.XMax, BasketInfo->Ball.YMin, BasketInfo->Ball.YMax, 255, 0, 255);//Ball Area pink
    ros_com->drawImageFunction(8, DrawMode::DrawObject, BasketInfo->Basket.XMin, BasketInfo->Basket.XMax, BasketInfo->Basket.YMin, BasketInfo->Basket.YMax, 255, 255, 0);//Basket Area yellow
    ros_com->drawImageFunction(9, DrawMode::DrawLine, BasketInfo->Basket.X, BasketInfo->Basket.X, 0, 240, 255, 255, 0);//Basket Vertical Midline yellow
}

void KidsizeStrategy::MoveHead(HeadMotorID ID, int Position, int Speed)//動頭(馬達編號，刻度，速度)
{
    ros_com->sendHeadMotor(ID,Position,Speed);
    tool->Delay(50);
    if(ID == HeadMotorID::HorizontalID) 
    {
        BasketInfo->HorizontalHeadPosition = Position;
    }
    else
    {
        BasketInfo->VerticalHeadPosition = Position;
    }
}

void KidsizeStrategy::MoveContinuous(int mode)//連續步態，根據ini檔裡面的參數來調整x,y,z,theda,IMU
{
    if(BasketInfo->Robot_State == Initialization)
    {
        walk_con->setWalkParameterInit(BasketInfo->ContinuousStep[mode].ContinuousInit.InitX, BasketInfo->ContinuousStep[mode].ContinuousInit.InitY, BasketInfo->ContinuousStep[mode].ContinuousInit.InitZ, 
                                       BasketInfo->ContinuousStep[mode].ContinuousInit.InitTheta);//連續步態初始值
        walk_con->setWalkParameterMax(3000, 2000, 0, 16);//連續步態上限
        walk_con->setWalkParameterMin(-3000, -2000, 0, -16);//連續步態下限
        walk_con->setWalkParameterExp(BasketInfo->ContinuousStep[mode].ContinuousMove.ExpX, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpY, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpZ, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpTheta);//連續步態期望值
        walk_con->setWalkParameterOneAddValueAndPeriod(BasketInfo->ContinuousStep[mode].ContinuousMove.AddX, BasketInfo->ContinuousStep[mode].ContinuousMove.AddY, BasketInfo->ContinuousStep[mode].ContinuousMove.AddZ, BasketInfo->ContinuousStep[mode].ContinuousMove.AddTheta, 
                                                         BasketInfo->AddPeriod);//連續步態x,y,z,theda，每?毫秒+多少期望值
    }
    else if(BasketInfo->ContinuousStep[mode].ContinuousMove.ChangeMode)//絕對連續步態，值會根據ini檔中的值輸入
    {
        walk_con->setWalkParameterExp(BasketInfo->ContinuousStep[mode].ContinuousMove.ExpX, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpY, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpZ, BasketInfo->ContinuousStep[mode].ContinuousMove.ExpTheta);
        walk_con->setWalkParameterOneAddValueAndPeriod(BasketInfo->ContinuousStep[mode].ContinuousMove.AddX, BasketInfo->ContinuousStep[mode].ContinuousMove.AddY, BasketInfo->ContinuousStep[mode].ContinuousMove.AddZ, BasketInfo->ContinuousStep[mode].ContinuousMove.AddTheta, 
                                                         BasketInfo->AddPeriod);
    }
    else//相對連續步態，值會根據ini檔中的原地踏步的值去做更改
    {
        walk_con->setWalkParameterExp(BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpX+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpX, BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpY+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpY,
                                        BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpZ+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpZ, BasketInfo->ContinuousStep[ContinuousStay].ContinuousMove.ExpTheta+BasketInfo->ContinuousStep[mode].ContinuousMove.ExpTheta);
        walk_con->setWalkParameterOneAddValueAndPeriod(BasketInfo->ContinuousStep[mode].ContinuousMove.AddX, BasketInfo->ContinuousStep[mode].ContinuousMove.AddY, BasketInfo->ContinuousStep[mode].ContinuousMove.AddZ, BasketInfo->ContinuousStep[mode].ContinuousMove.AddTheta, 
                                                         BasketInfo->AddPeriod);
    }
}

void KidsizeStrategy::InversePerspective()//逆透視法測距
{
    if (BasketInfo->Basket.size > Basketfarsize)
    {
        BasketInfo->MiddleAngle = atan(((double)BasketInfo->VisionMiddle / (double)BasketInfo->RobotSearchBasketHeight)) + BasketInfo->FeedBackError * Deg2Rad;
        BasketInfo->BAngle = atan((double)BasketInfo->ScreenButtom / (double)BasketInfo->RobotSearchBasketHeight);
        BasketInfo->AAngle = BasketInfo->MiddleAngle - BasketInfo->BAngle;

        if(BasketInfo->Basket.YMax > 120)
        {
            BasketInfo->dyAngle = atan2((double)((BasketInfo->Basket.YMax - 120) * tan((double)BasketInfo->AAngle)), 120);
            BasketInfo->BasketAngle = BasketInfo->MiddleAngle - BasketInfo->dyAngle;
        }
        else
        {	
            BasketInfo->dyAngle = atan2((double)((120 - BasketInfo->Basket.YMax) * tan((double)BasketInfo->AAngle)), 120);
            BasketInfo->BasketAngle = BasketInfo->MiddleAngle + BasketInfo->dyAngle;
        }

        BasketInfo->Distancenew = abs((double)BasketInfo->RobotSearchBasketHeight * tan((double)BasketInfo->BasketAngle)) + BasketInfo->Error;
    }
    else
    {
        BasketInfo->Distancenew = 80;
    }
}

void KidsizeStrategy::Triangulation()//三角測量測距
{
    MoveHead(HeadMotorID::HorizontalID,2048, 200);
    image();
    while(abs(BasketInfo->Basket.Y - 120) > 3)//頭部上下轉動直到對準籃框的水平中心線
    {
        if(BasketInfo->Basket.Y == 0)
        {
            MoveHead(HeadMotorID::VerticalID,1800, 200);
        } 
        else if((BasketInfo->Basket.Y - 120) > 0)
        {
            MoveHead(HeadMotorID::VerticalID,BasketInfo->VerticalHeadPosition - 1, 200);
        }
        else if((BasketInfo->Basket.Y - 120) < 0)
        {
            MoveHead(HeadMotorID::VerticalID,BasketInfo->VerticalHeadPosition + 1, 200);
        }
        image();
        ROS_INFO("Basket Y = %d", BasketInfo->Basket.Y);
    }
    BasketInfo->HeadVerticalAngle = (double)(BasketInfo->VerticalHeadPosition - 1024) * Scale2Deg + BasketInfo->RobotStandFeedBack + BasketInfo->FeedBackError;
    ROS_INFO("VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
    ROS_INFO("HeadVerticalAngle = %lf", BasketInfo->HeadVerticalAngle);
    ROS_INFO("DistanceError = %lf", BasketInfo->DistanceError);
    ROS_INFO("robothigh = %lf", BasketInfo->RobotHeight);
    ROS_INFO("sin(BasketInfo->HeadVerticalAngle * Deg2Rad) = %lf", sin(BasketInfo->HeadVerticalAngle * Deg2Rad));
    ROS_INFO("tan(BasketInfo->HeadVerticalAngle * Deg2Rad) = %lf", tan(BasketInfo->HeadVerticalAngle * Deg2Rad));
    //ROS_INFO("Lcamara = %d", CameraHeight * cos(BasketInfo->HeadVerticalAngle * Deg2Rad));
    BasketInfo->Distancenew = (BasketInfo->RobotHeight + CameraHeight * sin(BasketInfo->HeadVerticalAngle * Deg2Rad)) * tan(BasketInfo->HeadVerticalAngle * Deg2Rad) + CameraHeight * cos(BasketInfo->HeadVerticalAngle * Deg2Rad) + BasketInfo->DistanceError + 15 ;
}

void KidsizeStrategy::image()//影像辨識，用於辨識球模or籃框模
{
    strategy_info->get_image_flag = true;
	ros::spinOnce();
    //初始值都先給0
    BasketInfo->Ball.size = 0;
    BasketInfo->Ball.XMin = 0;
    BasketInfo->Ball.XMax = 0;
    BasketInfo->Ball.YMin = 0;
    BasketInfo->Ball.YMax = 0;
    BasketInfo->Ball.X    = 0;
    BasketInfo->Ball.Y    = 0;
    BasketInfo->Basket.size = 0;
    BasketInfo->Basket.XMin = 0;
    BasketInfo->Basket.XMax = 0;
    BasketInfo->Basket.YMin = 0;
    BasketInfo->Basket.YMax = 0;
    BasketInfo->Basket.X    = 0;
    BasketInfo->Basket.Y    = 0;

    for( int i = 0 ; i < strategy_info->color_mask_subject_cnts[BasketInfo->Ballcolor] ;i++)//球模
    {
        if(strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].size > Ballfarsize && strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].size < 40000 )//球的物件特徵
        {
            BasketInfo->Ball.XMin = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].XMin ;
            BasketInfo->Ball.XMax = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].XMax ;
            BasketInfo->Ball.YMin = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].YMin ;
            BasketInfo->Ball.YMax = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].YMax ;
            BasketInfo->Ball.X    = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].X ;
            BasketInfo->Ball.Y    = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].Y ;
            BasketInfo->Ball.size = strategy_info->color_mask_subject[BasketInfo->Ballcolor][i].size;
        }
    }

    for (int i = 0; i < strategy_info->color_mask_subject_cnts[BasketInfo->Basketcolor];i++)//籃框模
    {
        if(strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].size > Basketfarsize)//籃框的物件特徵
        {
            BasketInfo->Basket.XMin = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMin ;
            BasketInfo->Basket.XMax = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].XMax ;
            BasketInfo->Basket.YMin = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMin;
            BasketInfo->Basket.YMax = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].YMax;
            BasketInfo->Basket.X    = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].X ;
            BasketInfo->Basket.Y    = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].Y ;
            BasketInfo->Basket.size = strategy_info->color_mask_subject[BasketInfo->Basketcolor][i].size;
        }
    }
}

void KidsizeStrategy::AreaSizeMeasure()//面積法測距
{
    double front = 0, back = 0, EstimatedDistance = 0;
    ROS_INFO("BasketInfo->Basket.size = %d", BasketInfo->Basket.size);
    if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[0])
    {
        ROS_INFO("40-50");
        front = 40;
        back = 50;
    }
    else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[1])
    {
        ROS_INFO("50-60");
        front = 50;
        back = 60;
    }
    else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[2])
    {
        ROS_INFO("60-70");
        front = 60;
        back = 70;
    }
    else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[3])
    {
        ROS_INFO("70-80");
        front = 70;
        back = 80;
    }
    else
    {
        ROS_INFO("80-90");
        front = 80;
        back = 90;
    }
    BasketInfo->Distancenew = back - (10 * (double)BasketInfo->Basket.size - BasketInfo->SizeOfDist[(int)((back-50)/10)]\
                            / (double)BasketInfo->SizeOfDist[(int)((front-50)/10)] - BasketInfo->SizeOfDist[(int)((back-50)/10)]);
    ROS_INFO("estimated distant = %f",BasketInfo->Distancenew);
}

void KidsizeStrategy::ComputeSpeed()//計算力道，利用權重算法，ex:距離52，50的力道*0.8+60的力道*0.2(類似這種概念)，
{
    ROS_INFO("---\tstart computing\t---");
    if (BasketInfo->Distancenew <= BasketInfo->dis35_x)								
    {
        ROS_INFO("dist <= 35");
        BasketInfo->weight_35 = 1.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->Distancenew >= BasketInfo->dis35_x) && (BasketInfo->Distancenew < BasketInfo->dis40_x))		
    {
        ROS_INFO("35 <= dist < 40");
        BasketInfo->weight_35 = (BasketInfo->dis50_x - BasketInfo->Distancenew) / (BasketInfo->dis50_x - BasketInfo->dis40_x);
        BasketInfo->weight_40 = (BasketInfo->Distancenew - BasketInfo->dis40_x) / (BasketInfo->dis50_x - BasketInfo->dis40_x);
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->Distancenew >= BasketInfo->dis40_x) && (BasketInfo->Distancenew < BasketInfo->dis50_x))		
    {
        ROS_INFO("40 <= dist < 50");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = (BasketInfo->dis50_x - BasketInfo->Distancenew) / (BasketInfo->dis50_x - BasketInfo->dis40_x);
        BasketInfo->weight_50 = (BasketInfo->Distancenew - BasketInfo->dis40_x) / (BasketInfo->dis50_x - BasketInfo->dis40_x);
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->Distancenew >= BasketInfo->dis50_x) && (BasketInfo->Distancenew < BasketInfo->dis60_x))		
    {
        ROS_INFO("50 <= dist < 60");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = (BasketInfo->dis60_x - BasketInfo->Distancenew) / (BasketInfo->dis60_x - BasketInfo->dis50_x);
        BasketInfo->weight_60 = (BasketInfo->Distancenew - BasketInfo->dis50_x) / (BasketInfo->dis60_x - BasketInfo->dis50_x);
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->Distancenew >= BasketInfo->dis60_x) && (BasketInfo->Distancenew < BasketInfo->dis70_x))		
    {
        ROS_INFO("60 <= dist < 70");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = (BasketInfo->dis70_x -BasketInfo-> Distancenew) / (BasketInfo->dis70_x - BasketInfo->dis60_x);
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = (BasketInfo->Distancenew - BasketInfo->dis60_x) / (BasketInfo->dis70_x - BasketInfo->dis60_x);
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->Distancenew >= BasketInfo->dis70_x) && (BasketInfo->Distancenew < BasketInfo->dis80_x))		
    {
        ROS_INFO("70 <= dist < 80");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = (BasketInfo->dis80_x - BasketInfo->Distancenew) / (BasketInfo->dis80_x - BasketInfo->dis70_x);
        BasketInfo->weight_80 = (BasketInfo->Distancenew - BasketInfo->dis70_x) / (BasketInfo->dis80_x - BasketInfo->dis70_x);
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 0.0;
    }
    else if ((BasketInfo->Distancenew >= BasketInfo->dis80_x) && (BasketInfo->Distancenew < BasketInfo->dis90_x))		
    {
        ROS_INFO("80 <= dist < 90");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = (BasketInfo->dis90_x - BasketInfo->Distancenew) / (BasketInfo->dis90_x - BasketInfo->dis80_x);
        BasketInfo->weight_90 = (BasketInfo->Distancenew - BasketInfo->dis80_x) / (BasketInfo->dis90_x - BasketInfo->dis80_x);
    }
    else if (BasketInfo->Distancenew >= BasketInfo->dis90_x)						
    {
        ROS_INFO("dist >= 90");
        BasketInfo->weight_35 = 0.0;
        BasketInfo->weight_40 = 0.0;
        BasketInfo->weight_50 = 0.0;
        BasketInfo->weight_60 = 0.0;
        BasketInfo->weight_61 = 0.0;
        BasketInfo->weight_70 = 0.0;
        BasketInfo->weight_71 = 0.0;
        BasketInfo->weight_80 = 0.0;
        BasketInfo->weight_81 = 0.0;
        BasketInfo->weight_90 = 1.0;
    }
    ROS_INFO("BasketInfo->weight_35 = %d",BasketInfo->weight_35);
    ROS_INFO("BasketInfo->dis35speed = %d",BasketInfo->dis35speed);
    ROS_INFO("BasketInfo->weight_40 = %d",BasketInfo->weight_40);
    ROS_INFO("BasketInfo->dis40speed = %d",BasketInfo->dis40speed);
    ROS_INFO("BasketInfo->weight_50 = %d",BasketInfo->weight_50);
    ROS_INFO("BasketInfo->dis50speed = %d",BasketInfo->dis50speed);
    ROS_INFO("BasketInfo->weight_60 = %d",BasketInfo->weight_60);
    ROS_INFO("BasketInfo->dis60speed = %d",BasketInfo->dis60speed);
    ROS_INFO("BasketInfo->weight_70 = %d",BasketInfo->weight_70);
    ROS_INFO("BasketInfo->dis70speed = %d",BasketInfo->dis70speed);
    ROS_INFO("BasketInfo->weight_80 = %d",BasketInfo->weight_80);
    ROS_INFO("BasketInfo->dis80speed = %d",BasketInfo->dis80speed);
    ROS_INFO("BasketInfo->weight_90 = %d",BasketInfo->weight_90);
    ROS_INFO("BasketInfo->dis90speed = %d",BasketInfo->dis90speed);
    BasketInfo->disspeed =  BasketInfo->weight_35*BasketInfo->dis35speed + BasketInfo->weight_40*BasketInfo->dis40speed + BasketInfo->weight_50*BasketInfo->dis50speed + BasketInfo->weight_60*BasketInfo->dis60speed + BasketInfo->weight_70*BasketInfo->dis70speed + BasketInfo->weight_71*BasketInfo->dis71speed + BasketInfo->weight_80*BasketInfo->dis80speed + BasketInfo->weight_81*BasketInfo->dis81speed + BasketInfo->weight_90*BasketInfo->dis90speed;
    ROS_INFO("---\tfinish computing, the speed is %d\t---",BasketInfo->disspeed);
}

void KidsizeStrategy::SelectBaseLine()//以測距後的值來判斷BasketVerticalBaseLine
{
    if(BasketInfo->Distancenew > 80)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine80;
	}
	else if(BasketInfo->Distancenew > 70)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine70;
	}
	else if(BasketInfo->Distancenew > 60)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine60;
	}
	else if(BasketInfo->Distancenew > 50)
	{
		BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine50;
	}
    else
    {
        BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine;
    }

    if(BasketInfo->RobotPosition == BigGOAhead)//看球的初始方向在哪，以此來考慮是否要幫baseline加補償值
    {
        BasketInfo->BasketVerticalBaseLine = BasketInfo->BasketVerticalBaseLine;
    }
    else if(BasketInfo->RobotPosition == TurnLeft)//同上
    {
        BasketInfo->BasketVerticalBaseLine += 0;
    }
    else if(BasketInfo->RobotPosition == TurnRight)//同上
    {
        BasketInfo->BasketVerticalBaseLine += 0;
    }

}

void KidsizeStrategy::FindballInitial()//找球時，頭的初始位置
{
    MoveHead(HeadMotorID::VerticalID, 1645, 200);//1623
    MoveHead(HeadMotorID::HorizontalID, 2395, 200);//2643
}

void KidsizeStrategy::FindballHead()//尋找場上的球
{
    if(BasketInfo->Ball.size > Ballfarsize)//找到球了，開始追蹤球
    {
        ROS_INFO("FIND BALL");
        BasketInfo->Robot_State = Trace_Ball;
    }
    else
    {
        switch (BasketInfo->HeadVerticalState)//頭垂直狀態，抬頭&&低頭
        {
            case HeadTurnNear:
                MoveHead(HeadMotorID::VerticalID, 1575, 200);       
                break;
            case HeadTurnClose:
                MoveHead(HeadMotorID::VerticalID, 1100, 200);       
                break;
        }
        switch(BasketInfo->HeadHorizontalState)
        {
            case HeadTurnRight://頭水平狀態，以右轉為例，轉到頭的刻度-HeadTurnSpeed<=HorizontalMinAngle時，水平狀態&&垂直狀態改變，以此循環，直到Ball.size > Ballfarsize
                if((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) > BasketInfo->HorizontalMinAngle)//1805
                {          
                    MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed, 200);
                }
                else if((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) <= BasketInfo->HorizontalMinAngle)
                {
                    BasketInfo->HeadVerticalState = HeadTurnClose;
                    BasketInfo->HeadHorizontalState = HeadTurnLeft;
                }
                break;

            case HeadTurnLeft:
                if((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) < BasketInfo->HorizontalMaxAngle)
                {           
                    MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed, 200);        
                }
                else if((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) >= BasketInfo->HorizontalMaxAngle)
                {
                    BasketInfo->HeadVerticalState = HeadTurnNear;   
                    BasketInfo->HeadHorizontalState = HeadTurnRight;
                }
                break;
        }
    }   
}

void KidsizeStrategy::TraceballHead()//頭追蹤球
{
    if(BasketInfo->Ball.size <= Ballfarsize)//沒找到球時切回Find_Ball
    {
        ROS_INFO("Miss Ball");
        walk_con->stopContinuous();
        tool->Delay(800);
        BasketInfo->StraightCatchFlag = true;
        BasketInfo->PreRotateFlag = false;
        BasketInfo->Robot_State = Find_Ball;
    }
    else
    {
        BasketInfo->BallMoveX = BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine;//可以當作與球baseline的差
        BasketInfo->BallMoveY = BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine;
        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BallMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
        BasketInfo->ErrorVerticalAngle = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BallMoveY / (double)RobotVisionHeight;//馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
        MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (BasketInfo->ErrorVerticalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);
        if(abs(BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine) <= BasketInfo->BallVerticalError && abs(BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine) <= BasketInfo->BallHorizontalError)//當誤差小於Error時狀態切到Goto_Ball
        {
            if(BasketInfo->StraightCatchFlag)
            {
                if(BasketInfo->HorizontalHeadPosition > (2048 - 150) && BasketInfo->HorizontalHeadPosition < (2048 + 300) && BasketInfo->VerticalHeadPosition <= BasketInfo->CatchBallLine && !walk_con->isStartContinuous())//&& BasketInfo->VerticalHeadPosition >= BasketInfo->backLine 不開啟步態且符合夾球範圍時，機器人不追蹤球直接進行夾球，避免撞到球
                {
                    std::printf("\033[0;33mCatch Ball\033[0m\n");
                    BasketInfo->PreRotateFlag = true; 
                    BasketInfo->ContinuousFlag = false;
                    BasketInfo->StoopFlag = true;
                    BasketInfo->CatchBallModeFlag = false;
                    BasketInfo->Robot_State = Goto_Ball;
                }
                BasketInfo->StraightCatchFlag = false;   
            }
            if(!BasketInfo->PreRotateFlag)//預先旋轉
            {
                if(!walk_con->isStartContinuous())
                {
                    walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態
                }
                BasketInfo->PreRotateFlag = true;
                BasketInfo->RobotPosition = BigGOAhead;
                std::printf("\033[0;33mBall at front side\033[0m\n");
                //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++            
                if(BasketInfo->HorizontalHeadPosition >= (2048 + 120))//機器人向左執行預先旋轉
                {
                    std::printf("\033[0;33mBall at left side\033[0m\n");
                    // MoveHead(HeadMotorID::HorizontalID, 2048, 500);
                    // tool->Delay(1500);
                    ros::spinOnce();
                    gettimeofday(&tstart, NULL);
                    gettimeofday(&tend, NULL);
                    timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                    while (timeuse <= 10000)
                    {
                        image();
                        if((BasketInfo->HorizontalHeadPosition < (2048 + 200)) || !strategy_info->getStrategyStart())//(BasketInfo->Ball.X >= 160  && BasketInfo->Ball.X != 0)機器人視野對轉球的垂直中心線就開始前進去追球
                        {
                            break;
                        }
                        MoveContinuous(ContinuousTurnLeft);
                        gettimeofday(&tend, NULL);
                        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                        BasketInfo->BallMoveX = BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine;//可以當作與球baseline的差
                        BasketInfo->BallMoveY = BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine;
                        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BallMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
                        BasketInfo->ErrorVerticalAngle = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BallMoveY / (double)RobotVisionHeight;//馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
                        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
                        MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (BasketInfo->ErrorVerticalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);
                    }
                    BasketInfo->RobotPosition = TurnLeft;                  
                }
                else if(BasketInfo->HorizontalHeadPosition <= (2048 - 120))//機器人向右執行預先旋轉
                {
                    std::printf("\033[0;33mBall at right side\033[0m\n");   
                    // MoveHead(HeadMotorID::HorizontalID, 2048, 500);
                    // tool->Delay(1500);
                    ros::spinOnce();
                    gettimeofday(&tstart, NULL);
                    gettimeofday(&tend, NULL);
                    timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                    while (timeuse <= 10000)
                    {
                        image();
                        if((BasketInfo->HorizontalHeadPosition > (2048 - 200)) || !strategy_info->getStrategyStart()) //(BasketInfo->Ball.X <= 160  && BasketInfo->Ball.X != 0) 
                        {
                            break;
                        }
                        MoveContinuous(ContinuousTurnRight);
                        gettimeofday(&tend, NULL);
                        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
                        BasketInfo->BallMoveX = BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine;//可以當作與球baseline的差
                        BasketInfo->BallMoveY = BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine;
                        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BallMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
                        BasketInfo->ErrorVerticalAngle = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BallMoveY / (double)RobotVisionHeight;//馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
                        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
                        MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (BasketInfo->ErrorVerticalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);
                    }
                    BasketInfo->RobotPosition = TurnRight;
                }
                else
                {
                    std::printf("\033[0;33mBall at front side\033[0m\n");
                }
                //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                if(BasketInfo->VerticalHeadPosition <= (BasketInfo->ContinuousSlowLine + 100))//當機器人的位置與球的放置位置太近時，會讓速度慢的區間變大
                {
                    BasketInfo->ContinuousSlowLine = BasketInfo->ContinuousSlowLine + 100;
                    std::printf("\033[0;33mBall at close side\033[0m\n");
                    tool->Delay(1500);
                }

            }
            if(BasketInfo->RestartFindBallFlag)//當球的位置小於夾球區間並且機器人預先旋轉向後退時，會使機器人無法夾球，因此重新找球與追蹤球
            {
                if(BasketInfo->VerticalHeadPosition <= BasketInfo->CatchBallLine && BasketInfo->StoopFlag)
                {
                    ROS_INFO("Restart");
                    walk_con->stopContinuous();
                    tool->Delay(1500);
                    BasketInfo->Robot_State = Find_Ball;
                }
                else
                {
                    BasketInfo->Robot_State = Goto_Ball;
                }
                BasketInfo->RestartFindBallFlag = false;
            } 
            else
            {
                BasketInfo->Robot_State = Goto_Ball;
            }
        }
        else
        {
            if(BasketInfo->HorizontalHeadPosition < (BasketInfo->HorizontalMinAngle + 200) || BasketInfo->HorizontalHeadPosition > (BasketInfo-> HorizontalMaxAngle - 200))//代表追蹤球的誤差過大，需重新找球並判斷是否預先
            {
                walk_con->stopContinuous();
                tool->Delay(1500);
                BasketInfo->StraightCatchFlag = true;
                BasketInfo->PreRotateFlag = false;
                BasketInfo->Robot_State = Find_Ball;
            }
            if(BasketInfo->VerticalHeadPosition < BasketInfo->VerticalMinAngle)//代表搜尋到不必要的範圍或快撞到機構
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalMinAngle, 200);
                BasketInfo->Robot_State = Find_Ball;
            }
            else if(BasketInfo->VerticalHeadPosition > BasketInfo->VerticalMaxAngle)//代表搜尋到不必要的範圍
            {
                MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalMaxAngle, 200);
                BasketInfo->Robot_State = Find_Ball;
            }
        }
    }
}

void KidsizeStrategy::TraceballBody()
{
    if(BasketInfo->ContinuousFlag)//機器人以連續步態去追蹤球
    {
        ROS_INFO("Catch Ball VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
        if(BasketInfo->Ball.size <= Ballfarsize)//漏球時，切回Find_Ball
        {
            ROS_INFO("Miss Ball");
            walk_con->stopContinuous();
            tool->Delay(1500);
            BasketInfo->StraightCatchFlag = true;
            BasketInfo->PreRotateFlag = false;
            BasketInfo->Robot_State = Find_Ball;
        }
        else if(BasketInfo->VerticalHeadPosition > BasketInfo->ContinuousSlowLine)//ContinuousSlowLine是連續步態減速線
        {
            ROS_INFO("Stand_1");
            if(!walk_con->isStartContinuous())
            {
                walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 130))//在這區間執行前進小左旋修正
            {
                std::printf("\033[0;33mTurn Left\033[0m\n");
                MoveContinuous(ContinuousSmallLeft);
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 130))//在這區間執行前進小右旋修正
            {
                std::printf("\033[0;33mTurn Right\033[0m\n");
                MoveContinuous(ContinuousSmallRight);
            }
            else
            {
                std::printf("\033[0;33mGo Ahead\033[0m\n");
                MoveContinuous(ContinuousForward);
            }
            BasketInfo->Robot_State = Trace_Ball;
        }
		else if(BasketInfo->VerticalHeadPosition <= BasketInfo->ContinuousSlowLine)
		{
            ROS_INFO("Stand_2");
            // if(BasketInfo->VerticalHeadPosition <= BasketInfo->backLine)
            // {
            //     ROS_INFO("Back");
            //     if(!walk_con->isStartContinuous())
            //     {
            //         walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);
            //     }
            //     else
            //     {
            //         ROS_INFO("Catch Ball Back");
            //         MoveContinuous(ContinuousBackward);
            //         BasketInfo->Robot_State = Trace_Ball;
            //     }
            // }
            // else 
            if(BasketInfo->VerticalHeadPosition <= BasketInfo->CatchBallLine)//&& BasketInfo->VerticalHeadPosition >= BasketInfo->backLine)
            {
                if(walk_con->isStartContinuous())//當要回到找球狀態時，關閉連續步態
                {
                    walk_con->stopContinuous();
                    tool->Delay(1500);
                }
                BasketInfo->ContinuousFlag = false;
                BasketInfo->StoopFlag = true;
            }
            else if(!walk_con->isStartContinuous())
            {
                walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 10))//在這區間執行原地小左旋修正
            {
                ROS_INFO("Catch Ball Left");
                MoveContinuous(ContinuousSmallTurnLeft);
                BasketInfo->Robot_State = Trace_Ball;
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 10))//在這區間執行原地小右旋修正
            {
                ROS_INFO("Catch Ball Right");
                MoveContinuous(ContinuousSmallTurnRight);
                BasketInfo->Robot_State = Trace_Ball;
            } 
            else
            {
                ROS_INFO("Catch Ball Small Foward");
                MoveContinuous(ContinuousSmallForward);
                BasketInfo->Robot_State = Trace_Ball;
            }
		}
	}
    else if(BasketInfo->StoopFlag)//機器人彎腰準備進行夾球
    { 
        if(BasketInfo->VerticalHeadPosition <= BasketInfo->backLine)//BasketInfo->HorizontalHeadPosition < (2048 - 300) || BasketInfo->HorizontalHeadPosition > (2048 + 300) ||        && BasketInfo->VerticalHeadPosition >= BasketInfo->backLine 不開啟步態且符合夾球範圍時，機器人不追蹤球直接進行夾球，避免撞到球
                {
                    BasketInfo->CatchBallModeFlag = true;
                }
        if(BasketInfo->Ball.size <= Ballfarsize)
        {
            ROS_INFO("Miss Ball");
            BasketInfo->Robot_State = Find_Ball;
        }
        else
        { 
            if(abs(BasketInfo->Ball.X - BasketInfo->BallVerticalBaseLine) <= BasketInfo->BallVerticalError && abs(BasketInfo->Ball.Y - BasketInfo->BallHorizontalBaseLine) <= BasketInfo->BallHorizontalError)
            {
                ROS_INFO("HorizontalHeadPosition = %d",BasketInfo->HorizontalHeadPosition);
                ROS_INFO("VerticalHeadPosition = %d",BasketInfo->VerticalHeadPosition);
                MoveHead(HeadMotorID::VerticalID, BasketInfo->CatchBallVerticalHeadPosition, 200);//夾球時的頭部刻度，在ini中更改，記得視野要垂直於地面
                MoveHead(HeadMotorID::HorizontalID, 2048, 200);
                tool->Delay(1000);
                ROS_INFO("Waistdown");
                ROS_INFO("CatchBallMode = %d",BasketInfo->CatchBallModeFlag);
                if(BasketInfo->CatchBallModeFlag)
                {
                    ros_com->sendBodySector(BB_WaistDown1);
                    tool->Delay(5000);
                    ros_com->sendBodySector(BB_WaistCatch1);
                    tool->Delay(1000);
                }
                else
                {
                    ros_com->sendBodySector(BB_WaistDown2);
                    tool->Delay(5000);
                    ros_com->sendBodySector(BB_WaistCatch2);
                    tool->Delay(1000);
                }
                tool->Delay(1500);
                BasketInfo->StoopFlag = false;
                BasketInfo->MoveFlag = true;
            }
            else
            {
                BasketInfo->Robot_State = Trace_Ball;
            }
        }
    }
    else if (BasketInfo->MoveFlag)//機器人進行夾球修正
    {
        ROS_INFO("Push ball");
        ROS_INFO("BasketInfo->Ball.Y = %d", BasketInfo->Ball.Y);
        if(BasketInfo->Ball.Y > BasketInfo->CatchBallYLine)
        {
            BasketInfo->count = BasketInfo->Ball.Y - BasketInfo->CatchBallYLine;
            BasketInfo->HandMove = BasketInfo->count * 2;//2為Pixel值換成刻度的數值，可藉由球在影像中pixel的改變量，去調整雙手轉動的刻度量(可更改)
            ROS_INFO("IN");
            if(BasketInfo->CatchBallModeFlag)
            {
                // ros_com->sendSingleMotor(5, (-1)*BasketInfo->HandMove, 10);
                // tool->Delay(1500);
                // ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 10);
                // tool->Delay(1500);
            }
            BasketInfo->OutReturnFlag = true;
        }
        else if(BasketInfo->Ball.Y <= BasketInfo->CatchBallYLine)
        {
            BasketInfo->count = BasketInfo->CatchBallYLine - BasketInfo->Ball.Y;
            BasketInfo->HandMove = BasketInfo->count * 1.7; //1.7
            ROS_INFO("OUT");
            ros_com->sendSingleMotor(5, (1)*BasketInfo->HandMove, 10);
            tool->Delay(1500);//200
            ros_com->sendSingleMotor(1, (-1)*BasketInfo->HandMove, 10);
            tool->Delay(1500);//500
            BasketInfo->InReturnFlag = true;
        }
        BasketInfo->MoveFlag = false;
        BasketInfo->GetBallFlag = true;
    }
    else if(BasketInfo->GetBallFlag)//夾球並回歸站立持球動作
    {
        MoveHead(HeadMotorID::VerticalID, 2048, 200);
        MoveHead(HeadMotorID::HorizontalID, 2048, 200);
        if(BasketInfo->CatchBallModeFlag == true)
        {
            ROS_INFO("Waist up");
            ros_com->sendBodySector(BB_WaistUp1);
            tool->Delay(5000);///////////////////fix////////////////////////////////////////////
            if(BasketInfo->InReturnFlag)
            {
                ros_com->sendSingleMotor(5, (-1)*BasketInfo->HandMove, 100);
                tool->Delay(1000);
                ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 100);
                tool->Delay(1000);
                BasketInfo->InReturnFlag = false;
            }
            tool->Delay(500);
            ros_com->sendBodySector(BB_WaistUpFeedBack2);
            tool->Delay(3000);
        }
        else if(BasketInfo->CatchBallModeFlag == false)
        {
            ROS_INFO("Waist up");
            ros_com->sendBodySector(BB_WaistUp2);
            tool->Delay(5000);//////////////////fix/////////////////////////////////////////////
            if(BasketInfo->InReturnFlag)
            {
                ros_com->sendSingleMotor(5, (-1)*BasketInfo->HandMove, 100);
                tool->Delay(1000);
                ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 100);
                ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 100);
                tool->Delay(1000);
                BasketInfo->InReturnFlag = false;
            }
            tool->Delay(500);
            ros_com->sendBodySector(BB_WaistUpFeedBack);
            tool->Delay(3000);
        }
        if(BasketInfo->OutReturnFlag)
        {
            // ros_com->sendSingleMotor(5, (1)*BasketInfo->HandMove, 100);//在夾球期間，手有往內or往外，所以在投球前要將其歸位
            // tool->Delay(1000);
            // ros_com->sendSingleMotor(1, (-1)*BasketInfo->HandMove, 100);
            // tool->Delay(1000);
            BasketInfo->OutReturnFlag = false;
        }
        else if(BasketInfo->InReturnFlag)
        {
            // ros_com->sendSingleMotor(5, (-1)*BasketInfo->HandMove, 100);
            // tool->Delay(1000);
            // ros_com->sendSingleMotor(1, (1)*BasketInfo->HandMove, 100);
            // tool->Delay(1000);
            BasketInfo->InReturnFlag = false;
        }
        ROS_INFO("Hands Back");
        tool->Delay(1500);
        // ros_com->sendBodySector(BB_WaistUpFeedBack);
        // tool->Delay(2000);
        //==========================================change===============================================
        // if(!BasketInfo->LeftHandUpFlag)//將左手抬起，避免壓到球柱
        // {
        //     ros_com->sendSingleMotor(4, 900, 100);
        //     tool->Delay(1500);
        //     BasketInfo->LeftHandUpFlag = true;  
        // }
        //==========================================change===============================================

        if(!walk_con->isStartContinuous())
        {
            walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);
        }
        BasketInfo->GetBallFlag = false;
        BasketInfo->TurnFlag = true;
	}
    else if(BasketInfo->TurnFlag)//轉向籃框
    {
		if(abs(BasketInfo->Basket.X - 160) <= 90 && strategy_info->getIMUValue().Yaw < 60 && strategy_info->getIMUValue().Yaw > -60)//籃框位於視野中央時進到Find_Target
		{
            ROS_INFO("Start Finding Basket");
            tool->Delay(500);
            BasketInfo->Robot_State = Find_Target;
		}
        if(BasketInfo->RobotPosition == TurnLeft)//根據機器人的RobotPositoin來判別現在該轉哪邊，因為是TurnLeft因此向右轉
        {
            ROS_INFO("IMU Value = %f", strategy_info->getIMUValue().Yaw);
            if(!BasketInfo->FaceBasketFlag)
            {
                if(strategy_info->getIMUValue().Yaw < -30)//當RobotPosition與IMU的Yaw值矛盾時，依據IMU值去做旋轉(藉由更改RobotPosition)
                {
                    ROS_INFO("Turn Left IMU");
                     BasketInfo->RobotPosition = TurnRight;
                }
                BasketInfo->FaceBasketFlag = true;
            }
            else
            { 
                ROS_INFO("Turn Right to Find Basket");
                MoveContinuous(ContinuousTurnRight);
            }
        }
        else if(BasketInfo->RobotPosition == TurnRight)//根據機器人的RobotPositoin來判別現在該轉哪邊，因為是TurnRight因此向左轉
        {
            ROS_INFO("IMU Value = %f", strategy_info->getIMUValue().Yaw);
            if(!BasketInfo->FaceBasketFlag)
            {
                if(strategy_info->getIMUValue().Yaw > 30)
                {
                    ROS_INFO("Turn Right IMU");
                    BasketInfo->RobotPosition = TurnLeft;
                }
                BasketInfo->FaceBasketFlag = true;
            }
            else
            { 
                ROS_INFO("Turn Left to Find Basket ");
                MoveContinuous(ContinuousTurnLeft);
            }
        }
        else if(BasketInfo->RobotPosition == BigGOAhead)
        {
            if(strategy_info->getIMUValue().Yaw > 0)//根據IMU的Yaw值執行原地右旋
            {
                ROS_INFO("Forward Turn Right");
                MoveContinuous(ContinuousTurnRight);
            }
            else if(strategy_info->getIMUValue().Yaw <= 0)//根據IMU的Yaw值執行原地左旋
            {
                ROS_INFO("Forward Turn left");
                MoveContinuous(ContinuousTurnLeft);
            }
        }
    }
}

void KidsizeStrategy::FindbasketHead()//跟FindballHead()相同概念
{
    if(walk_con->isStartContinuous())
    {
        ros::spinOnce();
        gettimeofday(&tstart, NULL);
        gettimeofday(&tend, NULL);
        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
        while (timeuse <= 500)//執行0.5秒的原地踏步，使機器人有好的收腳
        {
            MoveContinuous(ContinuousStay);
            ros::spinOnce();
            gettimeofday(&tend, NULL);
            timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
        }
        // walk_con->stopContinuous();
        // tool->Delay(3000);//1500+1500(為了穩定機器人的重心與使左手舉起，因而增加1.5秒的Delay)
    }
    if(BasketInfo->Basket.size > Basketfarsize)
	{
		ROS_INFO("Found Basket");
        BasketInfo->Robot_State = Trace_Target;
	}
    else
    {
        switch(BasketInfo->HeadHorizontalState)
        {
            case HeadTurnRight:
                if ((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) > BasketInfo->BasketHorizontalMinAngle)
                {
                    MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed, 200);
                }
                else if ((BasketInfo->HorizontalHeadPosition - BasketInfo->HeadTurnSpeed) <= BasketInfo->BasketHorizontalMinAngle)
                {
                    BasketInfo->HeadHorizontalState = HeadTurnLeft;
                }
                break;
            case HeadTurnLeft:
                if ((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) < BasketInfo->BasketHorizontalMaxAngle)
                {
                    MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed, 200);
                }
                else if((BasketInfo->HorizontalHeadPosition + BasketInfo->HeadTurnSpeed) >= BasketInfo->BasketHorizontalMaxAngle)
                {
                    BasketInfo->HeadHorizontalState = HeadTurnRight;
                }
                break;
        }
    }	
}

void KidsizeStrategy::TracebasketHead()
{
    if(BasketInfo->LayUpFlag)//上籃策略
    {
        if(BasketInfo->Basket.size <= Basketfarsize)//當找不到籃框時，重新進Find_Target
        {
            ROS_INFO("miss the basket");
            BasketInfo->Robot_State = Find_Target;
        }
        else
        {
            BasketInfo->BasketMoveX = BasketInfo->Basket.X - 160;//可以當作與籃框baseline的差
            BasketInfo->BasketMoveY = BasketInfo->Basket.Y - 120;
            BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
            BasketInfo->ErrorVerticalAngle = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BasketMoveY / (double)RobotVisionHeight;//馬達轉攝影機240pixel時轉的角度*與球baseline的差/240pixel,算出會得到角度
            MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle *TraceDegreePercent * 1 * Deg2Scale), 200);//再利用上面得到的角度來換算成刻度，來call   MoveHead()
            MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (BasketInfo->ErrorVerticalAngle * TraceDegreePercent * 1 * Deg2Scale), 200);
            BasketInfo->Robot_State = UP_Basket;
        }
	}
    else if(BasketInfo->FivePointFlag) //5 point 投籃策略
    {
        if(!walk_con->isStartContinuous())//開啟連續步態
        {
            walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態  
        }
        
        BasketInfo->BasketMoveX = (BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine);//跟上籃的追蹤籃框為同一方法
        BasketInfo->BasketMoveY = (BasketInfo->Basket.Y - BasketInfo->BasketHorizontalBaseLine);
        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX/(double)RobotVisionWidth;
        BasketInfo->ErrorVerticalAngle  = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BasketMoveY/(double)RobotVisionHeight;
        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle * TraceDegreePercent * 0.5 * Deg2Scale) , 200);
        MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (BasketInfo->ErrorVerticalAngle * TraceDegreePercent * 0.5 * Deg2Scale) , 200);
        if(BasketInfo->HorizontalHeadPosition >= (2048 - 10) && BasketInfo->HorizontalHeadPosition <= (2048 + 10) && BasketInfo->Basket.size <= (BasketInfo->SizeOfDist[3]+BasketInfo->SizeOfDist[4])/2 && BasketInfo->Basket.size > BasketInfo->SizeOfDist[4])// && BasketInfo->Basket.size >= BasketInfo->SizeOfDist[1] 
        {
            BasketInfo->Robot_State = Goto_Target;
        }
        else if(BasketInfo->Basket.size <= Basketfarsize)
        {
            ROS_INFO("Miss Basket");
            BasketInfo->Robot_State = Find_Target;
        }
        else
        {
            ROS_INFO("Adjust direction 1");
            ROS_INFO("Basket.X = %d", BasketInfo->Basket.X);
            if(BasketInfo->Basket.size < BasketInfo->SizeOfDist[4] )//籃框面積小於距離90時的籃框面積大小時，執行前進
            {
                ROS_INFO("Forward");
                MoveContinuous(ContinuousSmallForward);
            }
            else if(BasketInfo->Basket.size > (BasketInfo->SizeOfDist[3]+BasketInfo->SizeOfDist[4])/2)//籃框面積大於距離85時的籃框面積大小時，執行後退
            {
                ROS_INFO("Back");
                MoveContinuous(ContinuousBackward);
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 10))//在此區間執行小左旋修正
            {
                ROS_INFO("Turn Left");
                MoveContinuous(ContinuousSmallTurnLeft);
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 10))//在此區間執行小右旋修正
            {
                ROS_INFO("Turn Right");
                MoveContinuous(ContinuousSmallTurnRight);
            }
        }
    }
    else//投籃策略，這邊trace的概念跟traceball的概念一樣
    {
        if(!walk_con->isStartContinuous())//開啟連續步態
        {
            walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);//開始連續步態  
        }
        
        BasketInfo->BasketMoveX = (BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine);//跟上籃的追蹤籃框為同一方法
        BasketInfo->BasketMoveY = (BasketInfo->Basket.Y - BasketInfo->BasketHorizontalBaseLine);
        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX/(double)RobotVisionWidth;
        BasketInfo->ErrorVerticalAngle  = BasketInfo->ImgVerticalAngle * (double)BasketInfo->BasketMoveY/(double)RobotVisionHeight;
        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle * TraceDegreePercent * 0.5 * Deg2Scale) , 200);
        MoveHead(HeadMotorID::VerticalID, BasketInfo->VerticalHeadPosition - (BasketInfo->ErrorVerticalAngle * TraceDegreePercent * 0.5 * Deg2Scale) , 200);
        if(BasketInfo->HorizontalHeadPosition >= (2048 - 10) && BasketInfo->HorizontalHeadPosition <= (2048 + 10) &&  BasketInfo->Basket.size > (BasketInfo->SizeOfDist[1]+BasketInfo->SizeOfDist[2])/2 && BasketInfo->Basket.size <= BasketInfo->SizeOfDist[1]) // && BasketInfo->Basket.size >= BasketInfo->SizeOfDist[1] 
        {
            BasketInfo->Robot_State = Goto_Target;
        }
        else if(BasketInfo->Basket.size <= Basketfarsize)
        {
            ROS_INFO("Miss Basket");
            BasketInfo->Robot_State = Find_Target;
        }
        else
        {
            ROS_INFO("Adjust direction 1");
            ROS_INFO("Basket.X = %d", BasketInfo->Basket.X);
            if(BasketInfo->Basket.size < (BasketInfo->SizeOfDist[2]+BasketInfo->SizeOfDist[1])/2)//籃框面積小於距離65時的籃框面積大小時，執行前進
            {
                ROS_INFO("Forward");
                MoveContinuous(ContinuousSmallForward);
            }
            else if(BasketInfo->Basket.size > BasketInfo->SizeOfDist[1])//籃框面積大於距離55時的籃框面積大小時，執行後退
            {
                ROS_INFO("Back");
                MoveContinuous(ContinuousBackward);
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 10))//在此區間執行小左旋修正
            {
                ROS_INFO("Turn Left");
                MoveContinuous(ContinuousSmallTurnLeft);
            }
            else if(BasketInfo->HorizontalHeadPosition < (2048 - 10))//在此區間執行小右旋修正
            {
                ROS_INFO("Turn Right");
                MoveContinuous(ContinuousSmallTurnRight);
            }
        }
    }
}

void KidsizeStrategy::TracebasketBody()
{
    tool->Delay(1000);
    SelectBaseLine();
	if(BasketInfo->RoateFlag)
	{
        ROS_INFO("Adjust direction 2");
        ROS_INFO("HorizontalHeadPosition = %d", BasketInfo->HorizontalHeadPosition);
        if(BasketInfo->HorizontalHeadPosition >= (2048 - 100) && BasketInfo->HorizontalHeadPosition <= (2048 + 100))
		{
            ROS_INFO("Body aimed basket");
            if(walk_con->isStartContinuous())//當要到投籃狀態時，關閉連續步態
            {
                walk_con->stopContinuous();
                tool->Delay(1500);
            }           
            MoveHead(HeadMotorID::VerticalID, 1990, 200);
            //============================================
            // ROS_INFO("Ready to shoot!!");
		    // ros_com->sendBodySector(BB_RaiseHand);//舉手
		    // tool->Delay(4500);
            //============================================
            BasketInfo->DistanceError = strategy_info->getIMUValue().Yaw * BasketInfo->DistanceErrorCount;//為了修正籃框左右邊的高低誤差，用IMU的Yaw值進行補償(僅用於三角測量的測距)
            BasketInfo->RoateFlag = false;
            BasketInfo->WaistFlag = true;
		}
		else
		{
			BasketInfo->Robot_State = Trace_Target;
		}
	}//+ BasketInfo->weight_61*BasketInfo->dis61speed 
	else if (BasketInfo->WaistFlag)
	{
		/*
        if(BasketInfo->Basket.X > 160)
        {
            BasketInfo->Distancenew = AreaSizeMeasure();//+ (BasketInfo->Basket.X - 160)/24;
        }
        else if(BasketInfo->Basket.X <= 160)
        {
            BasketInfo->Distancenew = AreaSizeMeasure();// + (160 - BasketInfo->Basket.X)/48;
        }
		*/
		Triangulation();//三角測量
        SelectBaseLine();//根據測距得到的值來選擇要哪條baseline
        ROS_INFO("1.Distancenew : %f, BaseLine = %d", BasketInfo->Distancenew, BasketInfo->BasketVerticalBaseLine);
        ROS_INFO("BasketInfo->Basket.X = %d", BasketInfo->Basket.X);
        if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) > 0)//轉腰調整Basket.X與BasketVerticalBaseLine的誤差
		{
            ROS_INFO("RIGHT");
			ros_com->sendSingleMotor(9, (-1)*(BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine), 75);

		}
		else if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) < 0)
		{
            ROS_INFO("LEFT");
			ros_com->sendSingleMotor(9, BasketInfo->BasketVerticalBaseLine - BasketInfo->Basket.X, 75);
		}  
        tool->Delay(1000);
        
        SelectBaseLine();
        ROS_INFO("2.Distancenew : %f, BaseLine = %d", BasketInfo->Distancenew, BasketInfo->BasketVerticalBaseLine);
        ROS_INFO("-------------------------------------------------");
        if(abs(BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) <= BasketInfo->WaistError)//判斷機器人是否對準BasketVerticalBaseLine
        {
            ROS_INFO("Align the basketline %d", BasketInfo->BasketVerticalBaseLine);
            BasketInfo->WaistFlag = false;
            BasketInfo->ComputeFlag = true;
        }
    }
    else if(BasketInfo->ComputeFlag)
    {
		ROS_INFO("CompteSpeed");
        Triangulation();
        if(BasketInfo->FivePointFlag)
        {
            if (BasketInfo->Distancenew < BasketInfo->dis80_x)								
            {
                ComputeSpeed();
                BasketInfo->disspeed = BasketInfo->disspeed - BasketInfo->SpeedError;
            }
            else
            {
                BasketInfo->disspeed = BasketInfo->dis80speed + (BasketInfo->Distancenew - BasketInfo->dis80_x)*5;
            }
        }
        else if(!BasketInfo->LayUpFlag)
        {
            ComputeSpeed();//3 point 用權重計算投籃力道
        }
		
        if(BasketInfo->ReAimFlag)
        {
            SelectBaseLine();
            ROS_INFO("Reaimed BasketVerticalBaseLine = %d", BasketInfo->BasketVerticalBaseLine);
            while(abs(BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) > BasketInfo->WaistError)//當轉完腰後誤差大於WaistError時，就重新再轉一次
            {
                if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) > 0)
                {
                    ROS_INFO("RIGHT");
                    ros_com->sendSingleMotor(9, (-1)*7, 75);
                }
                else if((BasketInfo->Basket.X - BasketInfo->BasketVerticalBaseLine) < 0)
                {
                    ROS_INFO("LEFT");
                    ros_com->sendSingleMotor(9, 5, 75);
                }
                image();      
            }
            BasketInfo->ReAimFlag = true;
        }

        // ros_com->sendHandSpeed(BB_ShootingBall, BasketInfo->disspeed);//根據ComputeSpeed()得到的速度輸入到投籃的磁區中
        // ROS_INFO("BasketInfo->disspeed %d", BasketInfo->disspeed);

        BasketInfo->ComputeFlag = false;
        BasketInfo->RaiseFlag = true;
    }
    else if (BasketInfo->RaiseFlag)
	{
        ROS_INFO("Ready to shoot!!");
		ros_com->sendBodySector(BB_RaiseHand);//舉手
		tool->Delay(5000);
		BasketInfo->RaiseFlag = false;
		BasketInfo->ThrowBallFlag = true;
	}
	else if (BasketInfo->ThrowBallFlag)
	{
        ROS_INFO("Shoot!!");
        ros_com->sendHandSpeed(BB_ShootingBall, BasketInfo->disspeed);//射出去
        ros_com->sendBodySector(BB_ShootingBall);
		tool->Delay(2000);
        
		BasketInfo->ThrowBallFlag = false;         
		BasketInfo->Robot_State = End;

        std::printf("\033[0;33mDistancenew : %f\033[0m\n", BasketInfo->Distancenew);
        std::printf("\033[0;33mDisspeed : %d\033[0m\n", BasketInfo->disspeed);
        std::printf("\033[0;33mBaseLine : %d\033[0m\n", BasketInfo->BasketVerticalBaseLine);
        ROS_INFO("BasketInfo->Basket.size = %d", BasketInfo->Basket.size);

        ROS_INFO("END");
	}
}

void KidsizeStrategy::UPbasket()
{   
    ROS_INFO("VerticalHeadPosition = %d", BasketInfo->VerticalHeadPosition);
    if (BasketInfo->VerticalHeadPosition > BasketInfo->UpBasketStopLine)//利用頭度刻度來判斷上籃的停止地點，可在ini檔中做更改
	{        
		if (!walk_con->isStartContinuous())
		{ 
			walk_con->startContinuous((WalkingMode)BasketInfo->ContinuousStep[ContinuousStand].ContinuousInit.Mode, (SensorMode)IMUSet);
            ros::spinOnce();
            gettimeofday(&tstart, NULL);
            gettimeofday(&tend, NULL);
            timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
            while (timeuse <= 1000)//走1秒鐘的原地踏步，以避免機器人起步的晃動
            {
                MoveContinuous(ContinuousStay);
                ros::spinOnce();
                gettimeofday(&tend, NULL);
                timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
            }
		}
        else if(BasketInfo->VerticalHeadPosition >= BasketInfo->ContinuousSlowLine2)
        {
            if(BasketInfo->HorizontalHeadPosition < (2048 - 150))//執行快速前進大右旋修正
            {
                MoveContinuous(ContinuousFastBigRight);
            } 
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 150))//執行快速前進大左旋修正
            {
                MoveContinuous(ContinuousFastBigLeft);
            }
            else
            {  
                MoveContinuous(ContinuousFastForward);
            }
            BasketInfo->Robot_State = Trace_Target;
        }
		else if(BasketInfo->VerticalHeadPosition < BasketInfo->ContinuousSlowLine2)
        {
            if(BasketInfo->HorizontalHeadPosition < (2048 - 50))//執行快速前進小右旋修正
            {
                MoveContinuous(ContinuousFastSmallRight);
            }
            else if(BasketInfo->HorizontalHeadPosition > (2048 + 50))//執行快速前進小左旋修正
            {
                MoveContinuous(ContinuousFastSmallLeft);
            }
            BasketInfo->Robot_State = Trace_Target;
        }
	}
    else//當進入上籃範圍時，停止連續步態
    {
        if (walk_con->isStartContinuous())
        {
            walk_con->stopContinuous();
            tool->Delay(1500);  
            BasketInfo->Robot_State = SlamDunk_Basket;
        }
		else
		{
			BasketInfo->Robot_State = Trace_Target;
		}        
    }   
}

void KidsizeStrategy::SlamDunk()//灌籃
{   
    if(BasketInfo->HandUpFlag)
    {
        tool->Delay(1500);
        image();
        BasketInfo->BasketMoveX = BasketInfo->Basket.X - 160;//可以當作與籃框baseline的差
        BasketInfo->ErrorHorizontalAngle = BasketInfo->ImgHorizontalAngle * (double)BasketInfo->BasketMoveX / (double)RobotVisionWidth;//馬達轉攝影機320pixel時轉的角度*與球baseline的差/320pixel,算出會得到角度
        MoveHead(HeadMotorID::HorizontalID, BasketInfo->HorizontalHeadPosition - (BasketInfo->ErrorHorizontalAngle * Deg2Scale), 200);//再利用上面得到的角度來換算成刻度，來call MoveHead()
        ROS_INFO("Hand_UP");
        ROS_INFO("BasketInfo->Basket.X = %d", BasketInfo->Basket.X);
        ros_com->sendBodySector(BB_UpHand);
        tool->Delay(4500);
        BasketInfo->HandUpFlag = false;
        BasketInfo->SlamDunkFlag = true;

        ROS_INFO("turnwaistangle*1 = %f", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*1);
        ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 1, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        tool->Delay(2000);

        // if((BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)>=200)
        // {
        //     ROS_INFO("turnwaistangle*1.15 = %d", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*1.15);
        //     ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 1.15, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        //     tool->Delay(2000);
        // }
        // else if((BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)<200 && (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)>=150)
        // {
        //     ROS_INFO("turnwaistangle*1.25 = %f", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*1.25);
        //     ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 1.25, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        //     tool->Delay(2000);
        // }
        // else if((BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)<150 && (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)>=100)
        // {
        //     ROS_INFO("turnwaistangle*1.4 = %f", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*1.4);
        //     ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 1.4, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        //     tool->Delay(2000);
        // }
        // else if((BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)<100 && (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)>=50)
        // {
        //     ROS_INFO("turnwaistangle*2 = %f", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*2.2);
        //     ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 2, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        //     tool->Delay(2000);
        // }
        // else if((BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)<50 && (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)>0)
        // {
        //     ROS_INFO("turnwaistangle*2.5 = %f", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*2.5);
        //     ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 2.5, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        //     tool->Delay(2000);
        // }
        // else if((BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)<0 && (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)>-200)
        // {
        //     ROS_INFO("turnwaistangle*1 = %f", (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle)*1);
        //     ros_com->sendSingleMotor(9, (BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle) * 1, 50);//將當前得水平刻度數值減去定值，計算出轉腰所需的轉動刻度，定值可在ini檔中做修改   *0.9  
        //     tool->Delay(2000);
        // }
    }
    else if(BasketInfo->SlamDunkFlag)
    {
        ros_com->sendBodySector(BB_SlamDunk);
        tool->Delay(5000);
        ROS_INFO("Slam Dunk Is End!!!!!!!!!!!!!!");
        std::printf("\033[0;33m VerticalHeadPosition : %d\033[0m\n", BasketInfo->VerticalHeadPosition);
        std::printf("\033[0;33m Horizontal Move Degree : %d\033[0m\n", BasketInfo->HorizontalHeadPosition - BasketInfo->SlamDunkHorizontalAngle);
        BasketInfo->SlamDunkFlag = false;
        BasketInfo->Robot_State = End;
    }
}
