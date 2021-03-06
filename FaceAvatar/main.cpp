﻿#pragma once 
#include "kinectframe.h"
#include "cootrans.h"

CvCapture *pCapture;

////////记录按下的键盘/////////////////////////////////////////////////////////////
char Key[256];
////////记录手柄状态/////////////////////////////////////////////////////////////
DINPUT_JOYSTATE input;

//记录表情
int face;

//kinect相关
KinectFrame bonedata;//实例一个kinect对象
Joint *userJoints = NULL;//骨骼数据
float RotateMae = 0, RotateUe = 0, RotateCe = 0;
IFaceFrameResult *userFace = NULL;//


///////////////////////len相关数据//////////////////////////
double lenwx = 0, lenwy = -24, lenwz = 50, lenro = 0,//len世界坐标系坐标
lencx = 0, lency = -24, lencz = 50,// len相机坐标系坐标
facefudu = 0, //表情变化程度
lenro2 = 0;
VECTOR lenW = { 0, -24, 50 }, lenC = { 0, -24, 50 };
lenPoseWorld lenposew; //传len世界坐标

roboControl old_robocnt;
lenPoseWorld old_lenposew;

Mat cameraframe;
BASEIMAGE BaseImage;

////////////////////kaito相关数据////////////////////////
double kaitowx = 10, kaitowy = -24, kaitowz = 70, kaitowang = 0, oldkaitowx = 10, oldkaitowz = 70;//kaito世界坐标系坐标
kaitoPoseWorld kaitoposew; //接收kaito世界坐标



double Ang = 0, //手柄摇杆倒下方向
joyz = 0, joydz = 0;//记录摇杆直接得到的实际值

roboControl robocnt; //传机器人控制数据


////////模型ID,动画ID，相机图片ID/////////////////////////////////////////////////////////////
int ModelHandleLen, ModelHandleKaito, AttachIndex, AttachIndex2, CameraHandle = -1;
float TotalTime, PlayTime, PlayTimekaito = 0;
unsigned int prevtime, nowtime;

////lrf的位置
//urg04lx	g_urg04lx;//接收ipc
//double camera_x, camera_y, camera_ang, timestep;

char camera_xst[50], camera_yst[50], camera_angst[50], lenrobowx[50], lenrobowy[50];//为存放想打印的数值

//aria估计的机器人位置
arobotpose arobotposenow;//接收aria位置信息
double camera_x2, camera_y2, camera_ang2;

VECTOR CameraPos_old = { 0, 0, 0 }, CameraPos = { 0, 0, -50 };//相机位置
double CameraAng_old = 0, CameraAng = 0;//相机方向
// thread
static unsigned __stdcall ipclistenThread(void *);

static unsigned __stdcall robotContlenrolThread(void *);

static unsigned __stdcall ipcpublishThread(void *);

static unsigned __stdcall cameraThread(void *);

static unsigned __stdcall kinectThread(void *);

// 接受lrf数据的
//void urg04lxHandler(MSG_INSTANCE ref, void *data, void *dummy);

// 接受alenrobot数据的
void arobotposeHandler(MSG_INSTANCE ref, void *data, void *dummy);
// 接受kaito位置
void kaitoposeHandler(MSG_INSTANCE ref, void *data, void *dummy);

VECTOR NOtoVector(int modelHandle, int jointNO);
VECTOR KinectToVector(int NO1, int NO2);
VECTOR KinectToVector2(int NO1, int NO2);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow){

	IPC_connect("MainAvatar");//连接时在ipc central中显示的模块信息
	//IPC_defineMsg(URG04LX_MSG, IPC_VARIABLE_LENGTH, URG04LX_MSG_FMT);//lrf数据通讯MSG
	IPC_defineMsg(ARPOSE_MSG, IPC_VARIABLE_LENGTH, ARPOSE_MSG_FMT);//aria机器人位置通讯MSG
	IPC_defineMsg(ROBOCONTROL_MSG, IPC_VARIABLE_LENGTH, ROBOCONTROL_MSG_FMT);//机器人行为控制通讯MSG
	IPC_defineMsg(LENPOSE_MSG, IPC_VARIABLE_LENGTH, LENPOSE_MSG_FMT);//len位置MSG
	IPC_defineMsg(KAITOPOSE_MSG, IPC_VARIABLE_LENGTH, KAITOPOSE_MSG_FMT);//kaito位置MSG


	//IPC_subscribeData(URG04LX_MSG, urg04lxHandler, NULL);//开始准备接受lrf数据，要是是传送端就不需要这个
	IPC_subscribeData(ARPOSE_MSG, arobotposeHandler, NULL);//开始准备接受aria位置数据，要是是传送端就不需要这个
	IPC_subscribeData(KAITOPOSE_MSG, kaitoposeHandler, NULL);//开始准备接受kaito位置数据，要是是传送端就不需要这个


	AllocConsole();//打开控制台
	freopen("CONIN$", "r+t", stdin); // 重定向 STDIN 
	freopen("CONOUT$", "w+t", stdout); // 重定向STDOUT 

	//创建多线程
	HANDLE   hth1, hth2, hth3, hth4, hth5;
	unsigned  uiThread1ID, uiThread2ID, uiThread3ID, uiThread4ID, uiThread5ID;

	hth1 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		ipclistenThread,
		NULL,           // arg list  
		0,
		&uiThread1ID);
	if (hth1 == 0)
		printfDx("Failed to create ipclisten thread 1\n");

	hth2 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		robotContlenrolThread,
		NULL,           // arg list  
		0,
		&uiThread2ID);
	if (hth2 == 0)
		printfDx("Failed to create lenrobotContlenrol thread 2\n");

	hth3 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		ipcpublishThread,
		NULL,           // arg list  
		0,
		&uiThread3ID);
	if (hth3 == 0)
		printfDx("Failed to create ipcpublish thread \n");

	hth4 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		cameraThread,
		NULL,           // arg list  
		0,
		&uiThread4ID);

	hth5 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		kinectThread,
		NULL,           // arg list  
		0,
		&uiThread5ID);
	if (hth5 == 0)
		printfDx("Failed to create kinect thread \n");


	SetUseZBuffer3D(TRUE);

	////////设置分辨率和色彩数量/////////////////////////////////////////////////////////////
	//SetGraphMode(1024, 700, 32);
	SetGraphMode(960, 640, 32);

	////////初始化并设为窗口模式/////////////////////////////////////////////////////////////
	if (ChangeWindowMode(TRUE) != DX_CHANGESCREEN_OK || DxLib_Init() == -1) return -1;

	////////画在里画面上/////////////////////////////////////////////////////////////
	SetDrawScreen(DX_SCREEN_BACK);

	////////读入模型/////////////////////////////////////////////////////////////
	ModelHandleLen = MV1LoadModel("lenmodel\\len.mv1");
	if (ModelHandleLen == -1)printfDx("无法载入len模型");
	ModelHandleKaito = MV1LoadModel("kaito\\kaito.pmx");
	if (ModelHandleKaito == -1)printfDx("无法载入kaito模型");


	//////////////模型放大系数/////////////////////////////////////////////////////////////
	//SetCameraDotAspect(1);
	MV1SetScale(ModelHandleLen, VGet(1.5f, 1.5f, 1.5f));
	MV1SetScale(ModelHandleKaito, VGet(1.5f, 1.5f, 1.5f));

	//缓存背景图片
	pCapture = cvCaptureFromFile("http://192.168.11.7:8080/?action=stream&amp;amp;type=.mjpg");//webcamera
	//pCapture = cvCaptureFromCAM(1); //local camera

	//读入表情
	int face;
	face = MV1SearchShape(ModelHandleLen, "ウィンク");

	//////////////前后描画范围/////////////////////////////////////////////////////////////
	SetCameraNearFar(1.0f, 200.0f);
	//////////////初始相机位置和注视点/////////////////////////////////////////////////////////////
	SetCameraPositionAndTarget_UpVecY(CameraPos, VGet(0.0f, 0.0f, 50.0f));

	float Fov=40.0f;
	SetupCamera_Perspective(Fov * DX_PI_F / 180.0f);
	Set3DSoundOneMetre(50.0f);

	//////////////载入动画，4个参数为别是模型id，动画id，哪个模型里面的动画，/////////////////////////////////////////////////////////////
	AttachIndex = MV1AttachAnim(ModelHandleLen, 0, -1, FALSE);
	if (AttachIndex == -1)printfDx("无法载入len走路动画");
	AttachIndex2 = MV1AttachAnim(ModelHandleKaito, 0, -1, FALSE);
	if (AttachIndex2 == -1)printfDx("无法载入kaito走路动画");


	//////////////此动画的再生时间/////////////////////////////////////////////////////////////
	TotalTime = MV1GetAttachAnimTotalTime(ModelHandleLen, AttachIndex);

	//再生时间
	PlayTime = 0.0f;


	prevtime = nowtime = GetTickCount(); //后面的函数返回的是此程序到此为止的执行时间

#pragma region 骨骼相?
	/////////?找名?XX的骨骼点//////////////////////////////////////////////////////////////////
	MATRIX M45, M89, M56, M90, M56_2, M90_2, M56_3, M90_3, M56_4, M90_4, M56_5, M90_5, M56_6, M90_6;

	int RightShoulderFrameNo, LeftShoulderFrameNo, RightElbowFrameNo, LeftElbowFrameNo, RightElbowFrameNo2, LeftElbowFrameNo2, RightElbowFrameNo3, LeftElbowFrameNo3, RightElbowFrameNo4, LeftElbowFrameNo4, RightElbowFrameNo5, LeftElbowFrameNo5, RightElbowFrameNo6, LeftElbowFrameNo6;
	RightShoulderFrameNo = MV1SearchFrame(ModelHandleLen, "右腕");
	cout << RightShoulderFrameNo << endl;
	LeftShoulderFrameNo = MV1SearchFrame(ModelHandleLen, "左腕");
	cout << LeftShoulderFrameNo << endl;
	RightElbowFrameNo = MV1SearchFrame(ModelHandleLen, "右ひじ");
	LeftElbowFrameNo = MV1SearchFrame(ModelHandleLen, "左ひじ");
	/*RightElbowFrameNo2 = MV1SearchFrame(ModelHandleLocal, "右手捩1");
	LeftElbowFrameNo2 = MV1SearchFrame(ModelHandleLocal, "左手捩1");
	RightElbowFrameNo3 = MV1SearchFrame(ModelHandleLocal, "右手捩2");
	LeftElbowFrameNo3 = MV1SearchFrame(ModelHandleLocal, "左手捩2");
	RightElbowFrameNo4 = MV1SearchFrame(ModelHandleLocal, "右手捩3");
	LeftElbowFrameNo4 = MV1SearchFrame(ModelHandleLocal, "左手捩3");
	RightElbowFrameNo5 = MV1SearchFrame(ModelHandleLocal, "右手捩4");
	LeftElbowFrameNo5 = MV1SearchFrame(ModelHandleLocal, "左手捩4");

	RightElbowFrameNo6 = MV1SearchFrame(ModelHandleLocal, "右袖連動");
	LeftElbowFrameNo6 = MV1SearchFrame(ModelHandleLocal, "左袖連動");*/


	VECTOR LocalPositionRS, LocalPositionLS, LocalPositionRE, LocalPositionLE, LocalPositionRE2, LocalPositionLE2, LocalPositionRE3, LocalPositionLE3, LocalPositionRE4, LocalPositionLE4, LocalPositionRE5, LocalPositionLE5, LocalPositionRE6, LocalPositionLE6;

	///////////将此点恢?到模型默?位置（如果?没改?可以不用此函数）//////////////////////////////////////////////////////////////////
	//MV1ResetFrameUserLocalMatrix(ModelHandleLocal, RightShoulderFrameNo);


	/////////得到此骨骼点当前位置向量//////////////////////////////////////////////////////////////////
	LocalPositionRS = NOtoVector(ModelHandleLen, RightShoulderFrameNo);
	LocalPositionLS = NOtoVector(ModelHandleLen, LeftShoulderFrameNo);

	LocalPositionRE = NOtoVector(ModelHandleLen, RightElbowFrameNo);
	LocalPositionLE = NOtoVector(ModelHandleLen, LeftElbowFrameNo);
	/*LocalPositionRE2 = NOtoVector(ModelHandleLocal, RightElbowFrameNo2);
	LocalPositionLE2 = NOtoVector(ModelHandleLocal, RightElbowFrameNo2);
	LocalPositionRE3 = NOtoVector(ModelHandleLocal, RightElbowFrameNo3);
	LocalPositionLE3 = NOtoVector(ModelHandleLocal, LeftElbowFrameNo3);
	LocalPositionRE4 = NOtoVector(ModelHandleLocal, RightElbowFrameNo4);
	LocalPositionLE4 = NOtoVector(ModelHandleLocal, LeftElbowFrameNo4);
	LocalPositionRE5 = NOtoVector(ModelHandleLocal, RightElbowFrameNo5);
	LocalPositionLE5 = NOtoVector(ModelHandleLocal, LeftElbowFrameNo5);
	LocalPositionRE6 = NOtoVector(ModelHandleLocal, RightElbowFrameNo6);
	LocalPositionLE6 = NOtoVector(ModelHandleLocal, LeftElbowFrameNo6);*/

	//testm1 = MV1GetFrameBaseLocalMatrix(ModelHandleLocal, RightShoulderFrameNo);//取得初期??矩?

	//MV1SetFrameUserLocalMatrix(ModelHandleLocal, RightShoulderFrameNo, testm1);	//指定のフレームの座標変換行列を設定する
#pragma endregion


	while (!ProcessMessage() && !ClearDrawScreen() && !GetHitKeyStateAll(Key) && !Key[KEY_INPUT_ESCAPE]){
		//    ↑消息处理        ↑清空画面              ↑键盘入力               ↑没有按下ESC

		// 取得当前时间
		nowtime = GetTickCount();
		/*printf("nowtime = %d,prevtime = %d\n", nowtime, prevtime);*/
#pragma region 加载相机图到背景
		
		if (CameraHandle == -1)
		{
			// 最初指定句柄是指向此图的
			CameraHandle = CreateGraphFromBaseImage(&BaseImage);
		}
		else
		{
			// 从第二回开始直接传
			ReCreateGraphFromBaseImage(&BaseImage, CameraHandle);
		}
		//描画，左上，右下的坐标，以及是否通透
		DrawExtendGraph(0, 0, 1024, 768, CameraHandle, FALSE);
#pragma endregion

		///画线，方便看行走范围
		DrawLine3D(VGet(30.0f, -24, 30.0f), VGet(-30.0f, -24, 30.0f), GetColor(0, 255, 0));
		DrawLine3D(VGet(30.0f, -24, 100), VGet(-30.0f, -24, 100), GetColor(0, 255, 0));
		DrawLine3D(VGet(30.0f, -24, 30), VGet(30.0f, -24, 100), GetColor(0, 255, 0));
		DrawLine3D(VGet(-30.0f, -24, 30), VGet(-30.0f, -24, 100), GetColor(0, 255, 0));

		////重新定位相机位置

		CameraPos.x = -camera_y2;
		CameraPos.y = 0;
		CameraPos.z = camera_x2;
		CameraAng = camera_ang2;
		/*CameraPos.x = 20;
		CameraPos.y = 0;
		CameraPos.z = 10;
		CameraAng = 110;*/
		//SetCameraPositionAndAngle(VGet(-100*camera_y, 0.0f, 100*camera_x), 0.0f, -camera_ang, 0.0f);//lrf
		SetCameraPositionAndAngle(CameraPos, 0.0f, CameraAng*DX_PI_F / 180, 0.0f);//alenrobot，坐标和三自由度转角

		///////////////此模型的素材数//////////////////////////////////////////////
		int MaterialNum = MV1GetMaterialNum(ModelHandleLen);
		for (int i = 0; i < MaterialNum; i++)
		{
			///////////////所有素材轮廓线粗细设成0//////////////////////////////////////////////
			MV1SetMaterialOutLineDotWidth(ModelHandleLen, i, 0);
			MV1SetMaterialOutLineWidth(ModelHandleLen, i, 0);
		}

		int MaterialNum2 = MV1GetMaterialNum(ModelHandleKaito);
		for (int i = 0; i < MaterialNum2; i++)
		{
			///////////////所有素材轮廓线粗细设成0//////////////////////////////////////////////
			MV1SetMaterialOutLineDotWidth(ModelHandleKaito, i, 0);
			MV1SetMaterialOutLineWidth(ModelHandleKaito, i, 0);
		}

#pragma region 手柄控制

		// 取得手柄状态
		GetJoypadDirectInputState(DX_INPUT_PAD1, &input);

		joyz = input.Z;
		joydz = input.Rz;

		lenC = ConvWorldPosToCameraPos(CameraPos, CameraAng*DX_PI_F / 180, lenW);//求相对坐标
		//cout <<"X = "<< lenC.x << "Z ="<<lenC.z << endl;//输出相对坐标

		if (joyz != 0 || joydz != 0){        //手柄有输入时才根据输入改相机坐标系下坐标

			Ang = atan2(fabs(joyz), fabs(joydz));

			if (input.Z < 0 && input.Rz <= 0){ //rz是0是向左，其他是左前
				lenro = 180 - Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Z < 0 && input.Rz > 0){ //左后
				lenro = Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Z > 0 && input.Rz > 0){ //右后
				lenro = 360 - Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Z > 0 && input.Rz <= 0){ //rz是0是右，其他是右前
				lenro = 180 + Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Rz == -1000){ //前
				lenro = 180 + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Rz == 1000){  //后
				lenro = 0 + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}

			lenro2 = DX_PI_F*(lenro - CameraAng) / 180; //角度转换成弧度,世界坐标角度回到相机坐标系
			//cout << lenro2 * 180 / DX_PI_F << endl; 
			lenwx = lenwx - 0.3*sin(DX_PI_F*(lenro) / 180); //
			lenwz = lenwz - 0.3*cos(DX_PI_F*(lenro) / 180);
			lenW = { lenwx, 0, lenwz };

		}

		if (input.Buttons[1] == 128)
		{
			robocnt.exitRobot = 1;
			exit(0);
		}
		else robocnt.exitRobot = 0;//退出程序

#pragma endregion

#pragma region kinect

		bonedata.Update();
		userJoints = bonedata.GetJointData();
		userFace = bonedata.GetFaceFrameRe();

		if (lenro2 * 180 / DX_PI_F >= 90 && lenro2 * 180 / DX_PI_F <= 270){  //正面照镜子
			/////////取得从某向量到某向量的??矩?//////////////////////////////////////////////////////////////////

			M45 = MMult(MGetRotVec2(VGet(-1, -1, 0), KinectToVector2(8, 9)), MGetTranslate(LocalPositionRS));
			M89 = MMult(MGetRotVec2(VGet(1, -1, 0), KinectToVector2(4, 5)), MGetTranslate(LocalPositionLS));
			M56 = MMult(MGetRotVec2(KinectToVector2(8, 9), KinectToVector2(9, 10)), MGetTranslate(LocalPositionRE));
			M90 = MMult(MGetRotVec2(KinectToVector2(4, 5), KinectToVector2(5, 6)), MGetTranslate(LocalPositionLE));

			/////////?行//////////////////////////////////////////////////////////////////
			MV1SetFrameUserLocalMatrix(ModelHandleLen, RightShoulderFrameNo, M45);
			MV1SetFrameUserLocalMatrix(ModelHandleLen, LeftShoulderFrameNo, M89);
			MV1SetFrameUserLocalMatrix(ModelHandleLen, RightElbowFrameNo, M56);
			MV1SetFrameUserLocalMatrix(ModelHandleLen, LeftElbowFrameNo, M90);
		}
		else{ //背面也?作一致
			///////////取得从某向量到某向量的??矩?//////////////////////////////////////////////////////////////////

			M45 = MMult(MGetRotVec2(VGet(-1, -1, 0), KinectToVector(4, 5)), MGetTranslate(LocalPositionRS));
			M89 = MMult(MGetRotVec2(VGet(1, -1, 0), KinectToVector(8, 9)), MGetTranslate(LocalPositionLS));
			M56 = MMult(MGetRotVec2(KinectToVector(4, 5), KinectToVector(5, 6)), MGetTranslate(LocalPositionRE));
			M90 = MMult(MGetRotVec2(KinectToVector(8, 9), KinectToVector(9, 10)), MGetTranslate(LocalPositionLE));

			/////////?行//////////////////////////////////////////////////////////////////
			MV1SetFrameUserLocalMatrix(ModelHandleLen, RightShoulderFrameNo, M45);
			MV1SetFrameUserLocalMatrix(ModelHandleLen, LeftShoulderFrameNo, M89);
			MV1SetFrameUserLocalMatrix(ModelHandleLen, RightElbowFrameNo, M56);
			MV1SetFrameUserLocalMatrix(ModelHandleLen, LeftElbowFrameNo, M90);
		}


#pragma endregion

		lenposew.x = lenwx;//更新要传的len位置
		lenposew.z = lenwz;
		lenposew.theta = lenro;

		MV1SetShapeRate(ModelHandleLen, face, facefudu); //表情相关

		if (PlayTime >= TotalTime)PlayTime = 3.0f;   //动画相关
		if (MV1SetAttachAnimTime(ModelHandleLen, AttachIndex, PlayTime) == -1)
			DrawString(50, 400, "设定len播放时间失败", GetColor(0, 255, 255));


		if (kaitoposew.move == 1)PlayTimekaito += (float)(nowtime - prevtime) / 1000.0f*30.0f;
		if (PlayTimekaito >= TotalTime)PlayTimekaito = 3.0f;
		/*oldkaitowx = kaitowx;
		oldkaitowz = kaitowz;*/

		if (MV1SetAttachAnimTime(ModelHandleKaito, AttachIndex2, PlayTimekaito) == -1)
			DrawString(50, 400, "设定kaito播放时间失败", GetColor(255, 0, 255));

		prevtime = nowtime;


		////////模型位置/////////////////////////////////////////////////////////////
		MV1SetPosition(ModelHandleLen, VGet(lenwx, lenwy, lenwz));
		MV1SetRotationXYZ(ModelHandleLen, VGet(0.0f, lenro * DX_PI_F / 180.0f, 0.0f));

		MV1SetPosition(ModelHandleKaito, VGet(kaitowx, kaitowy, kaitowz));
		MV1SetRotationXYZ(ModelHandleKaito, VGet(0.0f, kaitowang * DX_PI_F / 180.0f, 0.0f));
		/////////模型描画//////////////////////////////////////////////
		if (MV1DrawModel(ModelHandleLen) == -1)
			DrawString(50, 440, "len描画失败", GetColor(255, 255, 255));

		if (MV1DrawModel(ModelHandleKaito) == -1)
			DrawString(50, 440, "kaito描画失败", GetColor(255, 255, 255));

		//// 文字大小
		SetFontSize(20);

		//// 设置成黑色
		int Cr;
		Cr = GetColor(255, 255, 0);

		// 显示文字  先把数字转成字符型再描画
		sprintf(camera_xst, "%lf", CameraPos.x);
		sprintf(camera_yst, "%lf", CameraPos.z);
		sprintf(camera_angst, "%lf", camera_ang2);
		sprintf(lenrobowx, "%lf", lenwx);
		sprintf(lenrobowy, "%lf", lenwz);


		/*if (robocnt.needMov == 1)strcpy(needMovS, "move");
		else strcpy(needMovS, "stay");*/

		DrawString(0, 0, "camera pose", Cr);
		DrawString(0, 30, "x：", Cr);
		DrawString(0, 60, camera_xst, Cr);
		DrawString(0, 90, "z：", Cr);
		DrawString(0, 120, camera_yst, Cr);
		DrawString(0, 150, "ang：", Cr);
		DrawString(0, 180, camera_angst, Cr);

		DrawString(0, 250, "local Avatar pose", Cr);
		DrawString(0, 280, "x：", Cr);
		DrawString(0, 310, lenrobowx, Cr);
		DrawString(0, 340, "z：", Cr);
		DrawString(0, 370, lenrobowy, Cr);
		//DrawString(0, 400, needMovS, Cr);

		///////////////显示里画面//////////////////////////////////////////////
		ScreenFlip();
	}

	IPC_disconnect();
	DxLib_End();
	cvReleaseCapture(&pCapture);
	return 0;
}


void arobotposeHandler(MSG_INSTANCE ref, void *data, void *dummy) //接收相机位置
{
	arobotposenow = *(arobotpose *)data;
	camera_x2 = arobotposenow.x;
	camera_y2 = arobotposenow.y;
	camera_ang2 = arobotposenow.theta;
}

// 接受kaito的数据
void kaitoposeHandler(MSG_INSTANCE ref, void *data, void *dummy)
{
	kaitoposew = *(kaitoPoseWorld *)data;
	kaitowx = kaitoposew.x;
	kaitowz = kaitoposew.z;
	kaitowang = kaitoposew.theta;

}

//ipc监听间隔的线程
static unsigned __stdcall ipclistenThread(void *)
{
	while (1){
		IPC_listenWait(20);
		Sleep(50);
	}
}

static unsigned __stdcall ipcpublishThread(void *)
{

	while (1){
		//if (fabs(old_robocnt.dist - robocnt.dist) > 5.0 ||    // 控制数值变化超过一定范围才传送数据
		//	fabs(old_robocnt.theta - robocnt.theta) > 3.0) {
		IPC_publishData(ROBOCONTROL_MSG, &robocnt);
		//	old_robocnt = robocnt;
		//}

		if (joyz != 0 || joydz != 0)lenposew.move = 1;
		else lenposew.move = 0;
		IPC_publishData(LENPOSE_MSG, &lenposew);

		Sleep(200);
	}
}

//控制机器人的线程
static unsigned __stdcall robotContlenrolThread(void *)
{

	int lenrobostate;
	int dist = 0, angdui = 0;
	while (1){
		dist = sqrt(lenC.x*lenC.x + lenC.z*lenC.z);
		angdui = 180 * atan2(lenC.x, lenC.z) / DX_PI_F; //右侧在0和180，左侧0和-180
		robocnt.theta = angdui;
		robocnt.dist = dist;
		Sleep(50);
	}
}

static unsigned __stdcall cameraThread(void *)
{
	//将相机的图或一张图片加载到缓存
	while (1){
		cameraframe = cvQueryFrame(pCapture);
		//cameraframe = imread("bai.jpg");
		memset(&BaseImage, 0, sizeof(BASEIMAGE));
		CreateFullColorData(&BaseImage.ColorData);
		BaseImage.Width = cameraframe.cols;
		BaseImage.Height = cameraframe.rows;
		BaseImage.Pitch = cameraframe.step;
		BaseImage.GraphData = cameraframe.data;
		Sleep(50);
	}
}


static unsigned __stdcall kinectThread(void *)
{
	while (1){
		/*bonedata.Update();
		userJoints = bonedata.GetJointData();
		userFace = bonedata.GetFaceFrameRe();*/
		Sleep(100);
	}
}

VECTOR NOtoVector(int modelHandle, int jointNO){
	/////////?几个矩?和向量//////////////////////////////////////////////////////////////////
	MATRIX LocalMatrix;
	VECTOR LocalPosition;

	/////////得到某骨骼点当前矩?//////////////////////////////////////////////////////////////////
	LocalMatrix = MV1GetFrameLocalMatrix(modelHandle, jointNO);

	/////////得到此骨骼点当前位置向量//////////////////////////////////////////////////////////////////
	LocalPosition = VGet(LocalMatrix.m[3][0], LocalMatrix.m[3][1], LocalMatrix.m[3][2]);
	return LocalPosition;

}

VECTOR KinectToVector(int NO1, int NO2){
	return VGet(userJoints[NO2].Position.X - userJoints[NO1].Position.X, userJoints[NO2].Position.Y - userJoints[NO1].Position.Y, userJoints[NO2].Position.Z - userJoints[NO1].Position.Z);
}

VECTOR KinectToVector2(int NO1, int NO2){
	return VGet(userJoints[NO1].Position.X - userJoints[NO2].Position.X, userJoints[NO2].Position.Y - userJoints[NO1].Position.Y, userJoints[NO2].Position.Z - userJoints[NO1].Position.Z);
}