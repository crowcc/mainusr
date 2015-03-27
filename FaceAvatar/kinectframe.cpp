#pragma once  

#include "kinectframe.h"


// face property text layout offset in X axis
static const float c_FaceTextLayoutOffsetX = -0.1f;

// face property text layout offset in Y axis
static const float c_FaceTextLayoutOffsetY = -0.125f;

static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

int width = 0;
int height = 0;
unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

//cv::Mat bufferMat(height, width, CV_8UC4);
cv::Mat colorMat(height / 2, width / 2, CV_8UC4);



KinectFrame::KinectFrame() :
m_hWnd(NULL),
m_nStartTime(0),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_nNextStatusTime(0LL),
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pColorFrameReader(nullptr),
m_pBodyFrameReader(NULL)
{
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}
	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}
}


/// <summary>
/// Destructor
/// </summary>
KinectFrame::~KinectFrame()
{
	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}
	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// done with color frame reader
	SafeRelease(m_pColorFrameReader);

	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}


	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

HRESULT KinectFrame::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;
		IColorFrameSource* pColorFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
		else cout << "m_pKinectSensor->Open failed!" << endl;

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}
		else cout << "pColorFrameSource->Open failed!" << endl;
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}
		else cout << "m_pColorFrameReader->Open failed!" << endl;

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}
		else cout << "get_CoordinateMapper failed" << endl;

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
			}
		}
		if (!SUCCEEDED(hr))cout << "OpenReader failed" << endl;

		SafeRelease(pBodyFrameSource);
		SafeRelease(pColorFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{

		return E_FAIL;
	}

	return hr;
}


Joint* KinectFrame::GetJointData(){
	return joints;
}

IFaceFrameResult*  KinectFrame::GetFaceFrameRe(){
	return pFaceFrameResult;
}


void KinectFrame::ProcessFaces()
{
	HRESULT hr;
	IBody* ppBodies[BODY_COUNT] = { 0 };
	bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies));

	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		if (SUCCEEDED(hr))
		{
			if (bFaceTracked)
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				RectI faceBox = { 0 }; //上下左右4个整数
				PointF facePoints[FacePointType::FacePointType_Count]; //xy坐?


				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
				{
					hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);
					sizeLen = faceBox.Bottom - faceBox.Top;
					//cout << "up =   " << faceBox.Top << "    down =   " << faceBox.Bottom << "   left =   " << faceBox.Left << "   right =   " << faceBox.Right << endl;

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
					}

					if (SUCCEEDED(hr))
					{
						hr = GetFaceTextPositionInColorSpace(ppBodies[iFace], &headPose);
						//cout << "headPose.x = " << headPose.x << " headPose.y = " << headPose.y << endl;
					}

					//if (SUCCEEDED(hr))
					//{
					//	// draw face frame results
					//	m_pDrawDataStreams->DrawFaceFrameResults(iFace, &faceBox, facePoints, &faceRotation, faceProperties, &faceTextLayout);
					//	//第几个人的?，箱子，所有?的点，3个旋?角，属性（是不是笑之?），描画用
					//}
				}

				SafeRelease(pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}
		else cout << "NO FACE" << endl;

		SafeRelease(pFaceFrame);
	}

	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}
}

HRESULT KinectFrame::UpdateBodyData(IBody** ppBodies)
{
	HRESULT hr = E_FAIL;

	if (m_pBodyFrameReader != nullptr)
	{
		IBodyFrame* pBodyFrame = nullptr;
		hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
			for (int i = 0; i < BODY_COUNT; ++i)
			{
				IBody* pBody = ppBodies[i];
				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);

				if (SUCCEEDED(hr) && bTracked)
				{

					HandState leftHandState = HandState_Unknown;
					HandState rightHandState = HandState_Unknown;

					pBody->get_HandLeftState(&leftHandState);
					pBody->get_HandRightState(&rightHandState);

					hr = pBody->GetJoints(_countof(joints), joints);
				}
			}
		}
		SafeRelease(pBodyFrame);
	}

	return hr;
}

void KinectFrame::Update()
{
	if (!m_pColorFrameReader || !m_pBodyFrameReader)
	{
		return;
	}

	IColorFrame* pColorFrame = nullptr;
	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = nullptr;
		int nWidth = 0;
		int nHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = nullptr;

		hr = pColorFrame->get_RelativeTime(&nTime);

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));

			}
			else if (m_pColorRGBX) //执行的是这里
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);

			}
			else
			{
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr))
		{
			ProcessColor(pBuffer, nWidth, nHeight);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pColorFrame);
}

void KinectFrame::ProcessColor(RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	// Make sure we've received valid data
	if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
	{
		// Draw the data with OpenCV
		Mat ColorImage(nHeight, nWidth, CV_8UC4, pBuffer);
		bufferMat = ColorImage;
		//cout <<"H = "<< nHeight << "   W =  "<<nWidth << endl;
		//resize(ColorImage, bufferMat, Size(nWidth/2, nHeight/2));
		//imshow("ColorImage", bufferMat);
		//imshow("ColorImage", ColorImage);
	}
	ProcessFaces();
	//return *pBuffer;
}

HRESULT KinectFrame::GetFaceTextPositionInColorSpace(IBody* pBody, VECTOR* Faceposition) //身体数据和?色流?坐?
{
	HRESULT hr = E_FAIL;

	if (pBody != nullptr)
	{
		BOOLEAN bTracked = false;
		hr = pBody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			hr = pBody->GetJoints(_countof(joints), joints);
			if (SUCCEEDED(hr))
			{
				CameraSpacePoint headJoint = joints[JointType_Head].Position;  //利用骨骼数据得到的?的位置
				CameraSpacePoint textPoint =             //?的位置微修正
				{
					headJoint.X + c_FaceTextLayoutOffsetX,
					headJoint.Y + c_FaceTextLayoutOffsetY,
					headJoint.Z
				};

				ColorSpacePoint colorPoint = { 0 };
				hr = m_pCoordinateMapper->MapCameraPointToColorSpace(textPoint, &colorPoint);   //?才的位置是深度流里的，取得?色流里的?位置 ，?coP

				if (SUCCEEDED(hr))
				{
					(*Faceposition).x = colorPoint.X;
					(*Faceposition).y = colorPoint.Y;
				}
			}
		}
	}

	return hr;
}