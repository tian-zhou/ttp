#pragma once

// For OpenCV
#include "opencv2\core\core.hpp"
#include "opencv2\highgui\highgui.hpp"

// for debug
#ifdef _DEBUG
	#pragma comment (lib, "opencv_core249d")
	#pragma comment (lib, "opencv_imgproc249d")
	#pragma comment (lib, "opencv_highgui249d")
	#pragma comment (lib, "opencv_ml249d")
	#pragma comment (lib, "opencv_video249d")
	#pragma comment (lib, "opencv_features2d249d")
	#pragma comment (lib, "opencv_calib3d249d")
	#pragma comment (lib, "opencv_objdetect249d")
	#pragma comment (lib, "opencv_contrib249d")
	#pragma comment (lib, "opencv_legacy249d")
	#pragma comment (lib, "opencv_flann249d")
#endif

#ifdef NDEBUG
	#pragma comment (lib, "opencv_core249")
	#pragma comment (lib, "opencv_imgproc249")
	#pragma comment (lib, "opencv_highgui249")
	#pragma comment (lib, "opencv_ml249")
	#pragma comment (lib, "opencv_video249")
	#pragma comment (lib, "opencv_features2d249")
	#pragma comment (lib, "opencv_calib3d249")
	#pragma comment (lib, "opencv_objdetect249")
	#pragma comment (lib, "opencv_contrib249")
	#pragma comment (lib, "opencv_legacy249")
	#pragma comment (lib, "opencv_flann249")
#endif


// For general stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <fstream>

// For SocketWithWindows for Face
#include "KinectBody.h"

// For Mayo
#include "Myo.h"

// For EPOC
#include "EPOC.h"

// Constantly checking for kbhit for r
void CheckKbhit_R(bool &hit)
{
	cv::Mat img = cv::Mat(300, 300, CV_8UC3);
	while (1)
	{
		if (GetAsyncKeyState('R') < 0)
		{	
			img = cv::Scalar(0,0,255);
			hit = true;
		}
		else
		{
			img = cv::Scalar(0,255,0);
			hit = false;
		}
		cv::imshow("CheckKbhit_R", img);
		cv::waitKey(1);
	}
}

class Multimodal
{

public:
	// file writer to save data for further analysis
	std::ofstream fwriter;

	// vars related to kinect
	KinectSocket kinect;

	// vars related to myo
	MYO_PACKET myo_packet;

	// vars related to epoc
	EPOC_PACKET epoc_packet;

	// label
	bool label;

	// misc
	enum Modalities{MYO, FACE, EPOC};
	Modalities m;

	// modality chosen flag
	bool myoEnable, faceEnable, epocEnable;

	~Multimodal()
	{
		fwriter.close();
	}

	int init(bool _myoEnable, bool _faceEnable, bool _epocEnable)
	{
		myoEnable = _myoEnable;
		faceEnable = _faceEnable;
		epocEnable = _epocEnable;
		printf ("myo enabled? %i\nface enabled? %i\nepoc enabled? %i\n \n", myoEnable, faceEnable, epocEnable);

		// the man-crafted label to indicate the user's explicit request
		label = false;

		//***************************************//
		//  Init a thread to check for kbhit 'r' to record request //
		// boost::thread* kbhitRThread = new boost::thread(CheckKbhit_R, boost::ref(label));
		//***************************************//

		//***************************************//
		// Init Socket with Windows computer for Kinect for body and Face //
		//***************************************//
		if (faceEnable){
			if (kinect.InitSocket() == SOCKET_ERROR)
				return -1;

			boost::thread* FaceThread = new boost::thread(&KinectSocket::runFace, &kinect);
		}

		//***************************************//
		//        Init Myo                       //
		//***************************************//
		if (myoEnable){
			boost::thread* MyoThread = new boost::thread(run_myo, boost::ref(myo_packet));
		}

		//***************************************//
		//        Init EPOC                       //
		//***************************************//
		if (epocEnable){
			boost::thread* EpocThread = new boost::thread(runEpoc, boost::ref(epoc_packet));
		}
	
		// init filewriter
		InitWriter();
		return 1;

	}

	// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
	const std::string currentDateTime() {
		SYSTEMTIME st;
 
		//GetSystemTime(&st);
		//cout << SystemTime.wHour << endl;
		GetLocalTime(&st);
    
		char buffer[ 256 ];
		sprintf( buffer,
				 "%d_%02d_%02d_%02d_%02d_%02d_%03d", 
				 st.wYear,
				 st.wMonth, 
				 st.wDay,                      
				 st.wHour, 
				 st.wMinute, 
				 st.wSecond,
				 st.wMilliseconds ); 

		std::string str(buffer);
		return str;
	}


	// Init the fwrite
	void InitWriter()
	{
		bool newFileFlag = true;
		std::string logName;
		char* FullName = new char[50];
		if (newFileFlag)
		{
			// the user will enter a new log file name
			std::cout << "Please enter multimodal log file name: ";
			std::cin >> logName;
			std::cout << "Write data into: log/" << logName <<".txt"<< std::endl;
			char* ch_logName = new char[40];
			strcpy(ch_logName, logName.c_str());
			strcat(ch_logName, ".txt");
			strcpy(FullName, "log/");
			strcat(FullName, ch_logName);
		}
		else
		{
			// the default log file name
			FullName = "log/defaultLog.txt";
		}
		fwriter.open(FullName, std::ios::out); // |std::ios::app
		if (!fwriter.is_open())
			printf("!!!!Error!!!!! File writer cannot init... \n");
	}

	// write the raw signal into a local txt file for further analysis
	int WriteToTxt()
	{
		if (fwriter.is_open())
		{
			// write label into txt
			// fwriter << label << ' '; 

			// write time 
			fwriter << currentDateTime() << ' ';

			if (myoEnable)
				myo_packet.WriteToTxt(fwriter);
				
			if (faceEnable) 
				kinect._kinectpacket.WriteToTxt(fwriter);

			if (epocEnable)
				epoc_packet.WriteToTxt(fwriter);

			// end of line
			fwriter << std::endl; 

			return 0;
		}
		else
			return -1;
	}

	// print the packets from all the sensors
	int Print()
	{
		system("cls");
		if (faceEnable) 
			kinect._kinectpacket.Print();
		if (myoEnable ) 
			myo_packet.Print();
		if (epocEnable) 
			epoc_packet.Print();
		return 1;
	}
};