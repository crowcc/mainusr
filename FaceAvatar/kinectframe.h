#include "common.h"



class KinectFrame
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int       cColorWidth = 1920;
	static const int       cColorHeight = 1080;

	Joint joints[JointType_Count];
	IFaceFrameResult* pFaceFrameResult = nullptr;
	HRESULT hra;

public:

	Mat bufferMat;
	RGBQUAD*                m_pColorRGBX;

	VECTOR headPose;
	Vector4 faceRotation;
	DetectionResult faceProperties[FaceProperty::FaceProperty_Count];

	float sizeLen;

	KinectFrame();
	~KinectFrame();

	HRESULT InitializeDefaultSensor();
	Joint* GetJointData();
	IFaceFrameResult* GetFaceFrameRe();
	void ProcessColor(RGBQUAD* pBuffer, int nWidth, int nHeight);

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Main processing function
	/// </summary>
	void                    Update();

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	INT64                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;

	// Body reader
	IBodyFrameReader*       m_pBodyFrameReader;

	// Color reader
	IColorFrameReader*     m_pColorFrameReader;

	// Face sources
	IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];

	// Face readers
	IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];
	/// <summary>
	/// Handle new body data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="nBodyCount">body data count</param>
	/// <param name="ppBodies">body data in frame</param>
	/// </summary>
	void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);
	void                   ProcessFaces();
	HRESULT                UpdateBodyData(IBody** ppBodies);

	HRESULT                GetFaceTextPositionInColorSpace(IBody* pBody, VECTOR* Faceposition);
	/// <summary>
	/// Set the status bar message
	/// </summary>
	/// <param name="szMessage">message to display</param>
	/// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
	/// <param name="bForce">force status update</param>
	bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);


};

