#pragma once   //当被多次包含时也仅编译一次
#define _CRT_SECURE_NO_DEPRECATE
#include "DxLib.h"
#include<math.h>
#include <opencv2\opencv.hpp>
#include <ipc.h>
#include "urg.h"
#include "ipc_msg.h"
#include <process.h>
#include <iostream>
#include <Eigen/Dense>
#include <Kinect.h>  
#include <Kinect.Face.h>
// Windows Header Files
#include <windows.h>
#include <Shlobj.h>
#pragma comment ( lib, "kinect20.lib" )  

using namespace cv;
using namespace std;
using namespace Eigen;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}