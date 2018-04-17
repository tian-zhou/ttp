#pragma once


#include <stdio.h>
#include <Windows.h>
#include <vector>
#include <iostream>

#pragma comment(lib,"ws2_32.lib")
#define  PORT 4000
#define  IP_ADDRESS_Win10 "128.46.125.46"

#define M_PI 3.14159265358979323846

// for Kinect packet
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <assert.h>
#include <fstream>
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

	// mutex
	boost::mutex kinectMutex;

public:

	KinectPacket(){
		startTime = clock(); // time reference

		// init the member values to 0
		kinectMutex.lock();

		// time
		timepassed = 0;

		// face
		for (int8_t i = 0; i < FacePointType::FacePointType_Count; i ++)
		{
			facePoint[i].X = 0;
			facePoint[i].Y = 0;
		}
		boundingBox.Left = 0;
		boundingBox.Right = 0;
		boundingBox.Top = 0;
		boundingBox.Bottom = 0;		
		roll = 0;
		pitch = 0;
		yaw = 0;		
		for (int8_t i = 0; i < FaceProperty::FaceProperty_Count; i ++){
			faceProperty[i] = static_cast<DetectionResult>(0);
		}

		// body
		for (int8_t i = 0; i < JointType::JointType_Count; i ++)
		{
			joint[i].Position.X = 0;
			joint[i].Position.Y = 0;
			joint[i].Position.Z = 0;
		}
		leftHandState = static_cast<HandState>(0);
		rightHandState = static_cast<HandState>(0);
		lean.X = 0;
		lean.Y = 0;
		
		// audio beam
		angle = 0;
		confidence = 0;

		kinectMutex.unlock();

	}

	
	void ReadFromBuffer(const char* buf){
		std::string str4buf(buf, BUFFER_SIZE);
		std::istringstream iss (str4buf, std::istringstream::in);

		kinectMutex.lock();

		// time
		iss >> timepassed;

		// face
		for (int8_t i = 0; i < FacePointType::FacePointType_Count; i ++)
		{
			iss >> facePoint[i].X;
			iss >> facePoint[i].Y;
		}

		iss >> boundingBox.Left;
		iss >> boundingBox.Right;
		iss >> boundingBox.Top;
		iss >> boundingBox.Bottom;
		
		iss >> roll;
		iss >> pitch;
		iss >> yaw;
		
		int tmp;
		for (int8_t i = 0; i < FaceProperty::FaceProperty_Count; i ++){
			iss >> tmp;
			faceProperty[i] = static_cast<DetectionResult>(tmp);
		}

		// body
		for (int8_t i = 0; i < JointType::JointType_Count; i ++)
		{
			iss >> joint[i].Position.X;
			iss >> joint[i].Position.Y;
			iss >> joint[i].Position.Z;
		}

		iss >> tmp;
		leftHandState = static_cast<HandState>(tmp);
		iss >> tmp;
		rightHandState = static_cast<HandState>(tmp);

		iss >> lean.X;
		iss >> lean.Y;
		
		// audio beam
		iss >> angle;
		iss >> confidence;

		kinectMutex.unlock();
	}

	void Print()
	{
		printf ("Received Face packet: %s\n", buf);
	}


	void WriteToTxt(std::ofstream &fw)
	{
		// ***** For Face from Kinect ***** //
		// packet structure (frame rate is about 60Hz):
		
		char sep = ' ';

		// lock it
		kinectMutex.lock();

		// time
		dt = clock() - startTime;
		fw << (float)dt/CLOCKS_PER_SEC << sep;

		// face
		for (int8_t i = 0; i < FacePointType::FacePointType_Count; i ++)
		{
			fw << facePoint[i].X << sep;
			fw << facePoint[i].Y << sep;
		}

		fw << boundingBox.Left << sep;
		fw << boundingBox.Right << sep;
		fw << boundingBox.Top << sep;
		fw << boundingBox.Bottom << sep;
		
		fw << roll << sep;
		fw << pitch << sep;
		fw << yaw << sep;
		
		for (int8_t i = 0; i < FaceProperty::FaceProperty_Count; i ++)
			fw << faceProperty[i] << sep;

		// body
		for (int8_t i = 0; i < JointType::JointType_Count; i ++)
		{
			fw << joint[i].Position.X << sep;
			fw << joint[i].Position.Y << sep;
			fw << joint[i].Position.Z << sep;
		}

		fw << leftHandState << sep;
		fw << rightHandState << sep;

		fw << lean.X << sep;
		fw << lean.Y << sep;
		
		// audio beam
		fw << angle << sep;
		fw << confidence << sep;

		// unlock the mutex
		kinectMutex.unlock();
	}
};


class KinectSocket
{
public:
	WSADATA  Ws;
	SOCKET ClientSocket;
	struct sockaddr_in ClientAddr;
	int Ret;
	char SendBuffer[BUFFER_SIZE];
	char RcvBuffer[BUFFER_SIZE];

	// packet is a member var of the socket lcass
	KinectPacket _kinectpacket;

	KinectSocket(){
		Ret = 0;
	}

	~KinectSocket(){
		/* close socket */
		closesocket(ClientSocket);
		WSACleanup();
	}

	int InitSocket()
	{
		/* Init Windows Socket */
		if ( WSAStartup(MAKEWORD(2,2), &Ws) != 0 )
		{
			printf("Init Windows Socket Failed::%d\n", GetLastError());
			return -1;
		}

		/* Create Socket */
		ClientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if ( ClientSocket == INVALID_SOCKET )
		{
			printf("Create Socket Failed::%d\n", GetLastError());
			return -1;
		}

		ClientAddr.sin_family = AF_INET;
		ClientAddr.sin_addr.s_addr = inet_addr(IP_ADDRESS_Win10);
		ClientAddr.sin_port = htons(PORT);
		memset(ClientAddr.sin_zero, 0x00, 8);


		/* connect socket */
		printf("Waiting to connect to the Kinect Server...\n");
		while(1){
			Ret = connect(ClientSocket,(struct sockaddr*)&ClientAddr, sizeof(ClientAddr));
			if ( Ret == SOCKET_ERROR )
			{
				printf("Fail, Connect Error::%d, still waiting\n", GetLastError());
				//return -1;
				continue;
			}
			else
			{
				printf("--[Kinect Inited]--\n");
				return 0;
			}
		}
	}

	void FillBuf(const char * tmp)
	{
		strcpy(SendBuffer, tmp);
	}

	void ReceiveBuf()
	{
		Ret = recv(ClientSocket, RcvBuffer, BUFFER_SIZE, 0);
		if (Ret > 0){
			_kinectpacket.ReadFromBuffer(RcvBuffer);
		}
		else if (Ret == 0)
			printf ("Connection closing ...\n");
		else
		{
			printf("recv failed with error: %d \n",WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();

		}
	}
	void SendBuf()
	{
		/* send data to server */
		Ret = send(ClientSocket, SendBuffer, BUFFER_SIZE, 0);
		if ( Ret == SOCKET_ERROR )
			printf("Send Info Error::%d\n", GetLastError());
	}

	void runFace()
	{
		// measure the real-time FPS
		int FPSCount = 0;
		clock_t t, dt;
		t = clock(); // init the time 
		while (1) {
			// measure actual FPS
			// Debug: about 55 Hz when nobody, and 15 Hz when one person is present
			// Release: about 65Hz when nody, about 60Hz when one person is present

			//FPSCount++;
			//if (FPSCount == 50) // get an average from 50 intervals
			//{
			//	FPSCount = 0;
			//	dt = clock() - t; // get time difference
			//	printf ("Actual FPS for Kinect: %f\n", 50.0/(((float)dt)/CLOCKS_PER_SEC));
			//	t = clock(); // update the current clock
			//}

			ReceiveBuf();
		}

	}
};