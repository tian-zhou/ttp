#pragma once
#include <Kinect.h>
#include <Kinect.Face.h>
#include "opencv2/opencv.hpp"
#include <boost/thread/mutex.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include "KinectPacket.h"

template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL )
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class KinectBase
{
private:
	// general
	HRESULT hResult;
	std::string logName;

	// sensor
	IKinectSensor* pSensor;
	std::string id;

	// color
	IColorFrameSource* pColorSource;
	IColorFrameReader* pColorReader;
	IFrameDescription* pColorDescription;
	IColorFrame* pColorFrame;
	int colorWidth;
	int colorHeight;
	unsigned int colorBufferSize;
	cv::Mat colorBufferMat;
	cv::Mat colorMat;
	boost::mutex colorMutex;
	//cv::VideoWriter colorWriter;
	//std::string colorFilename;

	// depth
	IDepthFrameSource* pDepthSource;
	IDepthFrameReader* pDepthReader;
	IFrameDescription* pDepthDescription;
	IDepthFrame* pDepthFrame;
	int depthWidth;
	int depthHeight;
	unsigned int depthBufferSize;
	cv::Mat depthBufferMat;
	cv::Mat depthMat;
	boost::mutex depthMutex;
	UINT16 depthMinROI;
	UINT16 depthMaxROI;

	// coordinate mapper
	ICoordinateMapper* pCoordinateMapper;

	// body 
	IColorFrameSource* pColorSource4Body;
	IBodyFrameSource* pBodySource;
	IColorFrameReader* pColorReader4Body;
	IBodyFrameReader* pBodyReader;
	IFrameDescription* pBodyDescription;
	IColorFrame* pColorFrame4Body;
	IBodyFrame* pBodyFrame;
	ICoordinateMapper* pCoordinateMapper4Body;
	int width4Body;
	int height4Body;
	unsigned int bodyBufferSize;
	cv::Mat bodyBufferMat;
	cv::Mat bodyMat;
	cv::Mat bodyMatBGR;
	cv::Vec3b color[BODY_COUNT];
	boost::mutex bodyMutex;
	cv::VideoWriter bodyWriter;
	std::string bodyFilename;

	// face
	IFaceFrameSource* pFaceSource[BODY_COUNT];
	IFaceFrameReader* pFaceReader[BODY_COUNT];
	DWORD facefeatures;
	std::string property[FaceProperty::FaceProperty_Count];
	//cv::Mat faceBufferMat;
	//cv::Mat faceMat;

	// audio beam
	IAudioSource* pAudioSource;
	IAudioBeamFrameReader* pAudioReader;


public:
	// the socket packet to hold the values
	KinectPacket pac;

	KinectBase();
	~KinectBase();

	// sensor
	int InitSensor();
	int KillSensor();

	// color
	int InitColor();
	int UpdateColor();
	int UpdateColorForever();
	int ShowColor();
	int KillColor();
	cv::Mat GetColor();

	// depth
	int InitDepth();
	int SetDepthRange(UINT16 _depthMinROI, UINT16 _depthMaxROI);
	int UpdateDepth();
	int UpdateDepthForever();
	int ShowDepth();
	int KillDepth();
	cv::Mat GetDepth();

	// body (includes face)
	int InitBody();
	int UpdateBody();
	int UpdateBodyForever();
	int ShowBody();
	int KillBody();
	cv::Mat GetBodyMat();
	void DrawBone(const Joint* pJoints, const ColorSpacePoint* colorSpacePosition, JointType joint0, JointType joint1);
	void DrawBody(const Joint* pJoints, const ColorSpacePoint* colorSpacePosition);

	// coordinate mapper
	int InitCoordinateMapper();

	// audio beam
	int InitAudioBeam();
	int UpdateAudioBeam();
	int ForeverUpdateAudioBeam();
	int KillAudioBeam();

};

// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
inline void ExtractFaceRotationInDegrees( const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll );


