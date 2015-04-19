#pragma once 
#include "kinectframe.h"
#include "cootrans.h"

CvCapture *pCapture;

////////记录按下的键盘/////////////////////////////////////////////////////////////
char Key[256];
////////记录手柄状态/////////////////////////////////////////////////////////////
DINPUT_JOYSTATE input;

//记录表情
int face;

KinectFrame bonedata;//实例一个kinect对象

Joint *userJoints = NULL;//记录骨骼数据
float RotateMae = 0, RotateUe = 0, RotateCe = 0;
IFaceFrameResult *userFace = NULL;//记录脸数据

///////////////////////本地avatar相关数据//////////////////////////
double localAvaro = 0,//len世界坐标系坐标
localAvaro2 = 0,
facefudu = 0; //表情变化程度

VECTOR LocalAvaW = { 0, -24, 50 }, LocalAvaC = { 0, -24, 50 }; //求相机坐标系的作用是控制机器人
lenPoseWorld localAvaPoseW; //传len世界坐标

////////////////////远程avatar相关数据////////////////////////
double remoAvaro = 0;
VECTOR RemoAvaW = { 10, -24, 70 };
kaitoPoseWorld remoAvaPoseW; //接收kaito世界坐标

double Ang = 0, //手柄摇杆倒下方向
joyz = 0, joydz = 0;//记录摇杆直接得到的实际值

roboControl robocnt; //传机器人控制数据

HANDLE g_hMutexLopose, g_hMutexCamera, g_hMutexRemoAva, g_hMutexRoboCon;

////////模型ID,动画ID，相机图片ID/////////////////////////////////////////////////////////////
int ModelHandleLocal, ModelHandleRemote, AttachIndex, AttachIndex2, CameraHandle = -1;
float TotalTime, PlayTime, PlayTimekaito = 0;
unsigned int prevtime, nowtime;

////lrf的位置
//urg04lx	g_urg04lx;//接收ipc
//double camera_x, camera_y, camera_ang, timestep;

char camera_xst[50], camera_yst[50], camera_angst[50], localAvaWx[50], localAvaWz[50];//为存放想打印的数值

//aria估计的机器人位置
arobotpose arobotposenow;//接收aria位置信息
double camera_x2, camera_y2, camera_ang2;

VECTOR CameraPos_old = { 0, 0, 0 }, CameraPos = { 0, 0, 0 };//相机位置
double CameraAng_old = 0, CameraAng = 0;//相机方向

// thread
static unsigned __stdcall ipclistenThread(void *);

static unsigned __stdcall robotControlThread(void *);

static unsigned __stdcall ipcpublishThread(void *);

static unsigned __stdcall webcameraThread(void *);

static unsigned __stdcall kinectThread(void *);

// 接受lrf数据的
//void urg04lxHandler(MSG_INSTANCE ref, void *data, void *dummy);
// 接受alenrobot数据的
void arobotposeHandler(MSG_INSTANCE ref, void *data, void *dummy);
// 接受远程avatar位置
void remoAvaposeHandler(MSG_INSTANCE ref, void *data, void *dummy);

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
	IPC_subscribeData(KAITOPOSE_MSG, remoAvaposeHandler, NULL);//开始准备接受kaito位置数据，要是是传送端就不需要这个


	AllocConsole();//打开控制台
	freopen("CONIN$", "r+t", stdin); // 重定向 STDIN 
	freopen("CONOUT$", "w+t", stdout); // 重定向STDOUT 

	g_hMutexLopose = CreateMutex(NULL, FALSE, "LOPOSEMUTEX");
	if (g_hMutexLopose == NULL) {
		printf("CreateMutex(): Error\n");
	}
	g_hMutexCamera = CreateMutex(NULL, FALSE, "CAMERAMUTEX");
	if (g_hMutexCamera == NULL) {
		printf("CreateMutex(): Error\n");
	}
	g_hMutexRemoAva = CreateMutex(NULL, FALSE, "REMOAVAMUTEX");
	if (g_hMutexRemoAva == NULL) {
		printf("CreateMutex(): Error\n");
	}
	g_hMutexRoboCon = CreateMutex(NULL, FALSE, "ROBOCONMUTEX");
	if (g_hMutexRoboCon == NULL) {
		printf("CreateMutex(): Error\n");
	}

	//创建多线程
	HANDLE   hth1, hth2, hth3, hth4, hth5;
	unsigned  uiThread1ID, uiThread2ID, uiThread3ID, uiThread4ID, uiThread5ID;

	hth2 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		robotControlThread,
		NULL,           // arg list  
		0,
		&uiThread2ID);
	if (hth2 == 0)
		printfDx("Failed to create robotControl thread 2\n");

	hth1 = (HANDLE)_beginthreadex(NULL,       // security  
		0,            // stack size  
		ipclistenThread,
		NULL,           // arg list  
		0,
		&uiThread1ID);
	if (hth1 == 0)
		printfDx("Failed to create ipclisten thread 1\n");

	//Sleep(100);

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
		webcameraThread,
		NULL,           // arg list  
		0,
		&uiThread4ID);
	if (hth4 == 0)
		printfDx("Failed to create webcamera thread \n");

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
	SetGraphMode(1024, 700, 32);

	////////初始化并设为窗口模式/////////////////////////////////////////////////////////////
	if (ChangeWindowMode(TRUE) != DX_CHANGESCREEN_OK || DxLib_Init() == -1) return -1;

	////////画在里画面上/////////////////////////////////////////////////////////////
	SetDrawScreen(DX_SCREEN_BACK);

	////////读入模型/////////////////////////////////////////////////////////////
	ModelHandleLocal = MV1LoadModel("lenmodel\\len.mv1");
	if (ModelHandleLocal == -1)printfDx("无法载入len模型");
	ModelHandleRemote = MV1LoadModel("kaito\\kaito.pmx");
	if (ModelHandleRemote == -1)printfDx("无法载入kaito模型");


	//////////////模型放大系数/////////////////////////////////////////////////////////////
	//SetCameraDotAspect(1);
	MV1SetScale(ModelHandleLocal, VGet(1.5f, 1.5f, 1.5f));
	MV1SetScale(ModelHandleRemote, VGet(1.5f, 1.5f, 1.5f));

	//缓存背景图片
	pCapture = cvCaptureFromFile("http://192.168.11.7:8080/?action=stream&amp;amp;type=.mjpg");//webcamera
	//pCapture = cvCaptureFromCAM(1); //local camera

	//读入表情
	int face;
	face = MV1SearchShape(ModelHandleLocal, "ウィンク");

	//////////////前后描画范围/////////////////////////////////////////////////////////////
	SetCameraNearFar(1.0f, 200.0f);
	//////////////初始相机位置和注视点/////////////////////////////////////////////////////////////
	SetCameraPositionAndTarget_UpVecY(CameraPos, VGet(0.0f, 0.0f, 50.0f));


	//////////////载入动画，4个参数为别是模型id，动画id，哪个模型里面的动画，/////////////////////////////////////////////////////////////
	AttachIndex = MV1AttachAnim(ModelHandleLocal, 0, -1, FALSE);
	if (AttachIndex == -1)printfDx("无法载入len走路动画");
	AttachIndex2 = MV1AttachAnim(ModelHandleRemote, 0, -1, FALSE);
	if (AttachIndex2 == -1)printfDx("无法载入kaito走路动画");


	//////////////此动画的再生时间/////////////////////////////////////////////////////////////
	TotalTime = MV1GetAttachAnimTotalTime(ModelHandleLocal, AttachIndex);

	//再生时间
	PlayTime = 0.0f;


	prevtime = nowtime = GetTickCount(); //后面的函数返回的是此程序到此为止的执行时间



#pragma region 骨骼相关
	/////////寻找名为XX的骨骼点//////////////////////////////////////////////////////////////////
	MATRIX M45, M89, M56, M90, M56_2, M90_2, M56_3, M90_3, M56_4, M90_4, M56_5, M90_5, M56_6, M90_6;

	int RightShoulderFrameNo, LeftShoulderFrameNo, RightElbowFrameNo, LeftElbowFrameNo, RightElbowFrameNo2, LeftElbowFrameNo2, RightElbowFrameNo3, LeftElbowFrameNo3, RightElbowFrameNo4, LeftElbowFrameNo4, RightElbowFrameNo5, LeftElbowFrameNo5, RightElbowFrameNo6, LeftElbowFrameNo6;
	RightShoulderFrameNo = MV1SearchFrame(ModelHandleLocal, "右腕");
	cout << RightShoulderFrameNo << endl;
	LeftShoulderFrameNo = MV1SearchFrame(ModelHandleLocal, "左腕");
	cout << LeftShoulderFrameNo << endl;
	RightElbowFrameNo = MV1SearchFrame(ModelHandleLocal, "右ひじ");
	LeftElbowFrameNo = MV1SearchFrame(ModelHandleLocal, "左ひじ");
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

	///////////将此点恢复到模型默认位置（如果还没改过可以不用此函数）//////////////////////////////////////////////////////////////////
	//MV1ResetFrameUserLocalMatrix(ModelHandleLocal, RightShoulderFrameNo);


	/////////得到此骨骼点当前位置向量//////////////////////////////////////////////////////////////////
	LocalPositionRS = NOtoVector(ModelHandleLocal, RightShoulderFrameNo);
	LocalPositionLS = NOtoVector(ModelHandleLocal, LeftShoulderFrameNo);

	LocalPositionRE = NOtoVector(ModelHandleLocal, RightElbowFrameNo);
	LocalPositionLE = NOtoVector(ModelHandleLocal, LeftElbowFrameNo);
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

	//testm1 = MV1GetFrameBaseLocalMatrix(ModelHandleLocal, RightShoulderFrameNo);//取得初期变换矩阵

	//MV1SetFrameUserLocalMatrix(ModelHandleLocal, RightShoulderFrameNo, testm1);	//指定のフレームの座標変換行列を設定する
#pragma endregion


	bonedata.InitializeDefaultSensor();

	while (!ProcessMessage() && !ClearDrawScreen() && !GetHitKeyStateAll(Key) && !Key[KEY_INPUT_ESCAPE]){
		//    ↑消息处理        ↑清空画面              ↑键盘入力               ↑没有按下ESC

		// 取得当前时间
		nowtime = GetTickCount();
		/*printf("nowtime = %d,prevtime = %d\n", nowtime, prevtime);*/

		///画线，方便看行走范围
		DrawLine3D(VGet(30.0f, -24, 30.0f), VGet(-30.0f, -24, 30.0f), GetColor(0, 255, 0));
		DrawLine3D(VGet(30.0f, -24, 100), VGet(-30.0f, -24, 100), GetColor(0, 255, 0));
		DrawLine3D(VGet(30.0f, -24, 30), VGet(30.0f, -24, 100), GetColor(0, 255, 0));
		DrawLine3D(VGet(-30.0f, -24, 30), VGet(-30.0f, -24, 100), GetColor(0, 255, 0));

		////重新定位相机位置
		WaitForSingleObject(g_hMutexCamera, INFINITE);
		CameraPos.x = -camera_y2;
		CameraPos.y = 0;
		CameraPos.z = camera_x2;
		CameraAng = camera_ang2;
		ReleaseMutex(g_hMutexCamera);
		/*CameraPos.x = 20;
		CameraPos.y = 0;
		CameraPos.z = 10;
		CameraAng = 110;*/
		//SetCameraPositionAndAngle(VGet(-100*camera_y, 0.0f, 100*camera_x), 0.0f, -camera_ang, 0.0f);//lrf
		SetCameraPositionAndAngle(CameraPos, 0.0f, CameraAng*DX_PI_F / 180, 0.0f);//alenrobot，坐标和三自由度转角

		///////////////此模型的素材数//////////////////////////////////////////////
		int MaterialNum = MV1GetMaterialNum(ModelHandleLocal);
		for (int i = 0; i < MaterialNum; i++)
		{
			///////////////所有素材轮廓线粗细设成0//////////////////////////////////////////////
			MV1SetMaterialOutLineDotWidth(ModelHandleLocal, i, 0);
			MV1SetMaterialOutLineWidth(ModelHandleLocal, i, 0);
		}

		int MaterialNum2 = MV1GetMaterialNum(ModelHandleRemote);
		for (int i = 0; i < MaterialNum2; i++)
		{
			///////////////所有素材轮廓线粗细设成0//////////////////////////////////////////////
			MV1SetMaterialOutLineDotWidth(ModelHandleRemote, i, 0);
			MV1SetMaterialOutLineWidth(ModelHandleRemote, i, 0);
		}

#pragma region 手柄控制

		// 取得手柄状态
		GetJoypadDirectInputState(DX_INPUT_PAD1, &input);

		joyz = input.Z;
		joydz = input.Rz;

		LocalAvaC = ConvWorldPosToCameraPos(CameraPos, CameraAng*DX_PI_F / 180, LocalAvaW);//求相对坐标
		//cout <<"X = "<< LocalAvaC.x << "Z ="<<LocalAvaC.z << endl;//输出相对坐标

		if (joyz != 0 || joydz != 0){        //手柄有输入时才根据输入改相机坐标系下坐标

			Ang = atan2(fabs(joyz), fabs(joydz));

			if (input.Z < 0 && input.Rz <= 0){ //rz是0是向左，其他是左前
				localAvaro = 180 - Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Z < 0 && input.Rz > 0){ //左后
				localAvaro = Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Z > 0 && input.Rz > 0){ //右后
				localAvaro = 360 - Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Z > 0 && input.Rz <= 0){ //rz是0是右，其他是右前
				localAvaro = 180 + Ang * 180 / DX_PI_F + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Rz == -1000){ //前
				localAvaro = 180 + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}
			else if (input.Rz == 1000){  //后
				localAvaro = 0 + CameraAng;
				PlayTime += (float)(nowtime - prevtime) / 1000.0f*30.0f;
			}

			localAvaro2 = DX_PI_F*(localAvaro - CameraAng) / 180; //求相机坐标系下方向，是为了知道kinect反应哪个方向
			LocalAvaW.x = LocalAvaW.x - sin(DX_PI_F*(localAvaro) / 180); //
			LocalAvaW.z = LocalAvaW.z - cos(DX_PI_F*(localAvaro) / 180);

		}

		WaitForSingleObject(g_hMutexRoboCon, INFINITE);
		if (input.Buttons[1] == 128)robocnt.exitRobot = 1;
		else robocnt.exitRobot = 0;
		ReleaseMutex(g_hMutexRoboCon);

#pragma endregion
		WaitForSingleObject(g_hMutexLopose, INFINITE);
		localAvaPoseW.x = LocalAvaW.x;//更新要传的本地ava位置
		localAvaPoseW.z = LocalAvaW.z;
		localAvaPoseW.theta = localAvaro;
		ReleaseMutex(g_hMutexLopose);

#pragma region kinect
		
		if (localAvaro2 * 180 / DX_PI_F >= 90 && localAvaro2 * 180 / DX_PI_F <= 270){  //正面照镜子
			/////////取得从某向量到某向量的变换矩阵//////////////////////////////////////////////////////////////////

			M45 = MMult(MGetRotVec2(VGet(-1, -1, 0), KinectToVector2(8, 9)), MGetTranslate(LocalPositionRS));
			M89 = MMult(MGetRotVec2(VGet(1, -1, 0), KinectToVector2(4, 5)), MGetTranslate(LocalPositionLS));
			M56 = MMult(MGetRotVec2(KinectToVector2(8, 9), KinectToVector2(9, 10)), MGetTranslate(LocalPositionRE));
			M90 = MMult(MGetRotVec2(KinectToVector2(4, 5), KinectToVector2(5, 6)), MGetTranslate(LocalPositionLE));

			/////////执行//////////////////////////////////////////////////////////////////
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, RightShoulderFrameNo, M45);
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, LeftShoulderFrameNo, M89);
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, RightElbowFrameNo, M56);
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, LeftElbowFrameNo, M90);
		}
		else{ //背面也动作一致
			///////////取得从某向量到某向量的变换矩阵//////////////////////////////////////////////////////////////////

			M45 = MMult(MGetRotVec2(VGet(-1, -1, 0), KinectToVector(4, 5)), MGetTranslate(LocalPositionRS));
			M89 = MMult(MGetRotVec2(VGet(1, -1, 0), KinectToVector(8, 9)), MGetTranslate(LocalPositionLS));
			M56 = MMult(MGetRotVec2(KinectToVector(4, 5), KinectToVector(5, 6)), MGetTranslate(LocalPositionRE));
			M90 = MMult(MGetRotVec2(KinectToVector(8, 9), KinectToVector(9, 10)), MGetTranslate(LocalPositionLE));

			/////////执行//////////////////////////////////////////////////////////////////
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, RightShoulderFrameNo, M45);
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, LeftShoulderFrameNo, M89);
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, RightElbowFrameNo, M56);
			MV1SetFrameUserLocalMatrix(ModelHandleLocal, LeftElbowFrameNo, M90);
		}


#pragma endregion

		MV1SetShapeRate(ModelHandleLocal, face, facefudu); //表情相关

		if (PlayTime >= TotalTime)PlayTime = 3.0f;   //动画相关
		if (MV1SetAttachAnimTime(ModelHandleLocal, AttachIndex, PlayTime) == -1)
			DrawString(50, 400, "设定len播放时间失败", GetColor(0, 255, 255));


		if (remoAvaPoseW.move == 1)PlayTimekaito += (float)(nowtime - prevtime) / 1000.0f*30.0f;
		if (PlayTimekaito >= TotalTime)PlayTimekaito = 3.0f;

		if (MV1SetAttachAnimTime(ModelHandleRemote, AttachIndex2, PlayTimekaito) == -1)
			DrawString(50, 400, "设定kaito播放时间失败", GetColor(255, 0, 255));

		prevtime = nowtime;


		////////模型位置/////////////////////////////////////////////////////////////
		MV1SetPosition(ModelHandleLocal, VGet(LocalAvaW.x, LocalAvaW.y, LocalAvaW.z));
		MV1SetRotationXYZ(ModelHandleLocal, VGet(0.0f, localAvaro * DX_PI_F / 180.0f, 0.0f));

		WaitForSingleObject(g_hMutexRemoAva, INFINITE);
		MV1SetPosition(ModelHandleRemote, VGet(RemoAvaW.x, RemoAvaW.y, RemoAvaW.z));
		MV1SetRotationXYZ(ModelHandleRemote, VGet(0.0f, remoAvaro * DX_PI_F / 180.0f, 0.0f));
		ReleaseMutex(g_hMutexRemoAva);

		/////////模型描画//////////////////////////////////////////////
		if (MV1DrawModel(ModelHandleLocal) == -1)
			DrawString(50, 440, "len描画失败", GetColor(255, 255, 255));

		if (MV1DrawModel(ModelHandleRemote) == -1)
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
		sprintf(localAvaWx, "%lf", LocalAvaW.x);
		sprintf(localAvaWz, "%lf", LocalAvaW.z);

		DrawString(0, 0, "camera pose", Cr);
		DrawString(0, 30, "x：", Cr);
		DrawString(0, 60, camera_xst, Cr);
		DrawString(0, 90, "z：", Cr);
		DrawString(0, 120, camera_yst, Cr);
		DrawString(0, 150, "ang：", Cr);
		DrawString(0, 180, camera_angst, Cr);

		DrawString(0, 250, "local Avatar pose", Cr);
		DrawString(0, 280, "x：", Cr);
		DrawString(0, 310, localAvaWx, Cr);
		DrawString(0, 340, "z：", Cr);
		DrawString(0, 370, localAvaWz, Cr);

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

	WaitForSingleObject(g_hMutexCamera, INFINITE);
	camera_x2 = arobotposenow.x;
	camera_y2 = arobotposenow.y;
	camera_ang2 = arobotposenow.theta;
	ReleaseMutex(g_hMutexCamera);
}

// 接受kaito的数据
void remoAvaposeHandler(MSG_INSTANCE ref, void *data, void *dummy)
{
	remoAvaPoseW = *(kaitoPoseWorld *)data;

	WaitForSingleObject(g_hMutexRemoAva , INFINITE);
	RemoAvaW.x = remoAvaPoseW.x;
	RemoAvaW.z = remoAvaPoseW.z;
	remoAvaro = remoAvaPoseW.theta;
	ReleaseMutex(g_hMutexRemoAva);

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
		WaitForSingleObject(g_hMutexRoboCon, INFINITE);
		//if (fabs(old_robocnt.dist - robocnt.dist) > 5.0 ||    // 控制数值变化超过一定范围才传送数据
		//	fabs(old_robocnt.theta - robocnt.theta) > 3.0) {
		IPC_publishData(ROBOCONTROL_MSG, &robocnt);
		ReleaseMutex(g_hMutexRoboCon);
		//	old_robocnt = robocnt;
		//}

		if (joyz != 0 || joydz != 0){
			localAvaPoseW.move = 1;
		}
		else localAvaPoseW.move = 0;

		WaitForSingleObject(g_hMutexLopose, INFINITE);
		IPC_publishData(LENPOSE_MSG, &localAvaPoseW);
		ReleaseMutex(g_hMutexLopose);
		Sleep(100);
	}
}

//控制机器人的线程
static unsigned __stdcall robotControlThread(void *)
{

	int dist = 0, angdui = 0;
	while (1){
		//IPC_listenWait(20);
		dist = sqrt(LocalAvaC.x*LocalAvaC.x + LocalAvaC.z*LocalAvaC.z);
		angdui = 180 * atan2(LocalAvaC.x, LocalAvaC.z) / DX_PI_F; //右侧在0和180，左侧0和-180

		WaitForSingleObject(g_hMutexRoboCon, INFINITE);
		robocnt.theta = angdui;
		robocnt.dist = dist;
		ReleaseMutex(g_hMutexRoboCon);

	/*	cout << "robocnt.theta" << robocnt.theta << endl
			<< "robocnt.dist" << robocnt.dist << endl;*/
		Sleep(50);
	}
}

static unsigned __stdcall webcameraThread(void *)
{
	while (1){
		//将相机的图或一张图片加载到缓存
		Mat cameraframe;
		cameraframe = cvQueryFrame(pCapture);
		//cameraframe = imread("bai.jpg");

		BASEIMAGE BaseImage;

		memset(&BaseImage, 0, sizeof(BASEIMAGE));
		CreateFullColorData(&BaseImage.ColorData);
		BaseImage.Width = cameraframe.cols;
		BaseImage.Height = cameraframe.rows;
		BaseImage.Pitch = cameraframe.step;
		BaseImage.GraphData = cameraframe.data;


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
		Sleep(50);
	}
}


static unsigned __stdcall kinectThread(void *)
{
	while (1){
		bonedata.Update();
		userJoints = bonedata.GetJointData();
		userFace = bonedata.GetFaceFrameRe();
		Sleep(50);
	}
}

VECTOR NOtoVector(int modelHandle, int jointNO){
	/////////搞几个矩阵和向量//////////////////////////////////////////////////////////////////
	MATRIX LocalMatrix;
	VECTOR LocalPosition;

	/////////得到某骨骼点当前矩阵//////////////////////////////////////////////////////////////////
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