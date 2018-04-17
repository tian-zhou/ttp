#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <assert.h>
#define BUFFER_SIZE 2048 

class KinectPacket {
public:
	// time
	clock_t dt, startTime;
	float timepassed;

	// face related 
	PointF facePoint[FacePointType::FacePointType_Count];
	RectI boundingBox;
	int roll, pitch, yaw;
	DetectionResult faceProperty[FaceProperty::FaceProperty_Count];

	// body related
	Joint joint[JointType::JointType_Count];
	HandState leftHandState, rightHandState;
	PointF lean;

	// audio beam related
	float angle;  // radian [-0.872665f, 0.872665f]
	float confidence; // confidence [0.0f, 1.0f]

	// the char buffer for socket
	char buf[BUFFER_SIZE];

public:

	KinectPacket(){
		startTime = clock(); // time reference
	}

	void WriteToBuffer(){
		std::ostringstream oss;
		char sep = ' ';

		// time
		dt = clock() - startTime;
		timepassed = (float)dt/CLOCKS_PER_SEC;
		oss << timepassed << sep;

		// face
		for (int8_t i = 0; i < FacePointType::FacePointType_Count; i ++)
		{
			oss << facePoint[i].X << sep;
			oss << facePoint[i].Y << sep;
		}

		oss << boundingBox.Left << sep;
		oss << boundingBox.Right << sep;
		oss << boundingBox.Top << sep;
		oss << boundingBox.Bottom << sep;
		
		oss << roll << sep;
		oss << pitch << sep;
		oss << yaw << sep;
		
		for (int8_t i = 0; i < FaceProperty::FaceProperty_Count; i ++)
			oss << faceProperty[i] << sep;

		// body
		for (int8_t i = 0; i < JointType::JointType_Count; i ++)
		{
			oss << joint[i].Position.X << sep;
			oss << joint[i].Position.Y << sep;
			oss << joint[i].Position.Z << sep;
		}

		oss << leftHandState << sep;
		oss << rightHandState << sep;

		oss << lean.X << sep;
		oss << lean.Y << sep;
		
		// audio beam
		oss << angle << sep;
		oss << confidence << sep;
		
		// copy the stringstrem to a string then to a char array		
		strcpy(buf, oss.str().c_str());
	}
};
