#ifndef _LOADPARAMETER_H_
#define _LOADPARAMETER_H_
#include <ros/ros.h>
#include <ros/package.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "tku_libs/TKU_tool.h"
#include "strategy/basketballinfo.h"

using namespace std;
extern string ParameterPath;
class LoadParameter
{
public:
    LoadParameter() 
	{
		tool = ToolInstance::getInstance();
	};
	~LoadParameter()
	{
		ToolInstance::deleteInstance();
	};

    ToolInstance *tool;

    float readvalue(fstream &fin, string title,int mode);
	void initparameterpath();
    void LoadParameters();
    void TestParameters();

	string parameter_path = "N";

};

extern LoadParameter *Load;
#endif

