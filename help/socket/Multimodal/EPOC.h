//****************************************************************************
//**
//** Copyright 2014 by Emotiv. All rights reserved
//** Example 9 - EmoState And EEG Logger
//** This example logs both log Data and Affectiv Data from EmoEngine/EmoComposer
//** at the same time
//** Data will be logged in two output files : Affectiv_Data.csv and EEG_Data.csv
//****************************************************************************/
#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <stdexcept>
#include <cstdlib>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#endif
#ifdef __linux__
#include <unistd.h>
#endif
#include "EmoStateDLL.h"
#include "edk.h"
#include "edkErrorCode.h"

// the global mutex
boost::mutex epocMutex;

// define the structure for the packet of myo
struct EPOC_PACKET {
  	std::vector <double> EEG;
	std::vector <double> EmoState;
	void Print()
	{
		printf ("EEG Sigal:\n");
		for (int i = 0; i < EEG.size(); i++)
			std::cout <<EEG[i] << ' ';
		printf ("\n Emotion State: \n");
		for (int i = 0; i < EmoState.size(); i++)
			std::cout <<EmoState[i] << ' ';
	}	

	// ***** For EPOC EEG sensor ***** //
	// packet structure (each data takes one line, frame rate is 50Hz, set in the main function):
	void WriteToTxt(std::ofstream &fw)
	{	
		char sep = ' ';
		epocMutex.lock();
		for (int i = 0; i < 22; i ++)
			fw << EEG[i] << sep;
		for (int i = 0; i < 5; i ++)
			fw << EmoState[i] << sep;
		epocMutex.unlock();
	}
};




EE_DataChannel_t targetChannelList[] = {
    ED_COUNTER,
    ED_AF3, ED_F7, ED_F3, ED_FC5, ED_T7,
    ED_P7, ED_O1, ED_O2, ED_P8, ED_T8,
    ED_FC6, ED_F4, ED_F8, ED_AF4, ED_GYROX, ED_GYROY, ED_TIMESTAMP,
    ED_FUNC_ID, ED_FUNC_VALUE, ED_MARKER, ED_SYNC_SIGNAL
};

const char header[] = "COUNTER,AF3,F7,F3, FC5, T7, P7, O1, O2,P8" 
                      ", T8, FC6, F4,F8, AF4,GYROX, GYROY, TIMESTAMP, "   
                      "FUNC_ID, FUNC_VALUE, MARKER, SYNC_SIGNAL,";
const char affectivSuitesName[] = "Engagement,Frustration,Meditation,"
                                  "Excitement,Valence";


int runEpoc(EPOC_PACKET &_epocpacket) {
	_epocpacket.EEG.resize(22);
	_epocpacket.EmoState.resize(5);

	EmoEngineEventHandle eEvent = EE_EmoEngineEventCreate();
	EmoStateHandle eState = EE_EmoStateCreate();
	unsigned int userID = 0;
	const unsigned short composerPort = 1726;
	float secs = 1;
	unsigned int datarate = 0;
	bool readytocollect = false;
	int option = 1;
	int state = 0;

	
	try {
		
		if (EE_EngineConnect() != EDK_OK) {
			throw std::runtime_error("Emotiv Engine start up failed.");
		}
			
		
		DataHandle hData = EE_DataCreate();
		EE_DataSetBufferSizeInSec(secs);

		float affEngegement = 0,
			affFrus = 0,
			affMed = 0,
			affExcitement = 0,
			affValence = 0;

		while (!_kbhit()) {
			state = EE_EngineGetNextEvent(eEvent);
			EE_Event_t eventType;

			if (state == EDK_OK) {
				eventType = EE_EmoEngineEventGetType(eEvent);
				EE_EmoEngineEventGetUserId(eEvent, &userID);
				EE_EmoEngineEventGetEmoState(eEvent, eState);

				// Log the EmoState if it has been updated
				if (eventType == EE_UserAdded) {
					EE_DataAcquisitionEnable(userID, true);
					readytocollect = true;
					printf("--[Epoc Inited]-- \n");
				}

				if (readytocollect && (eventType == EE_EmoStateUpdated)) {
					unsigned int nSamplesTaken = 0;

					EE_DataUpdateHandle(0, hData);
					EE_DataGetNumberOfSample(hData, &nSamplesTaken);

					//std::cout << "Updated " << nSamplesTaken << std::endl;

					if (nSamplesTaken != 0) {
						double* data = new double[nSamplesTaken];
						for (int sampleIdx = 0; sampleIdx<(int)nSamplesTaken;
							++sampleIdx) {
							epocMutex.lock();
							for (int i = 0; i<sizeof(targetChannelList) /
								sizeof(EE_DataChannel_t); i++) {

								EE_DataGet(hData, targetChannelList[i], data,
									nSamplesTaken);
								_epocpacket.EEG[i] = data[sampleIdx];
								
							}
							epocMutex.unlock();

							affEngegement = ES_AffectivGetEngagementBoredomScore(eState);
							affFrus = ES_AffectivGetFrustrationScore(eState);
							affMed = ES_AffectivGetMeditationScore(eState);
							affExcitement = ES_AffectivGetExcitementShortTermScore(eState);
							affValence = ES_AffectivGetValenceScore(eState);
							epocMutex.lock();
							_epocpacket.EmoState[0] = affEngegement;
							_epocpacket.EmoState[1] = affFrus;
							_epocpacket.EmoState[2] = affMed;
							_epocpacket.EmoState[3] = affExcitement;
							_epocpacket.EmoState[4] = affValence;
							epocMutex.unlock();
							//printf("Engagement: %f, Frustration: %f, ...\n",
							//	affEngegement, affFrus);

						}

						delete[] data;
					}

					
				}
			}


			Sleep(100);
		}

		EE_DataFree(hData);

	}
	catch (const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		std::cout << "Press any key to exit..." << std::endl;
		getchar();
	}

	EE_EngineDisconnect();
	EE_EmoStateFree(eState);
	EE_EmoEngineEventFree(eEvent);

	return 0;
}


//class EPOC
//{
//public:
//	EmoEngineEventHandle eEvent;
//	EmoStateHandle eState;
//	unsigned int userID;
//	unsigned short composerPort;
//	float secs;
//	unsigned int datarate;
//	bool readytocollect;
//	int option;
//	int state;
//	double affEngegement, // they were float before, I just change them to double here for consistency
//        affFrus,
//        affMed ,
//        affExcitement,
//        affValence;
//
//	DataHandle hData;
//
//	std::string input;
//	//std::ofstream ofs; 
//	//std::ofstream ofs2; 
//
//	
//	// the epoc packet is a member of the EPOC class
//	EPOC_PACKET _epocpacket;
//
//	EPOC()
//	{
//		eEvent = EE_EmoEngineEventCreate();
//		eState = EE_EmoStateCreate();
//		composerPort						= 1726;
//		unsigned int userID					= 0;
//		float secs							= 1;
//		unsigned int datarate				= 0;
//		bool readytocollect					= false;
//		int option							= 1;
//		int state							= 0;
//		affEngegement = 0, // they were float before, I just change them to double here for consistency
//        affFrus = 0,
//        affMed = 0,
//        affExcitement = 0,
//        affValence = 0;
//
//	}
//
//	int Init()
//	{
//		_epocpacket.EEG.resize(22);
//		_epocpacket.EmoState.resize(5);
//
//		if (EE_EngineConnect() != EDK_OK) {
//            throw std::runtime_error("Emotiv Engine start up failed.");
//		}
//				
//		
//        //std::cout << "Start receiving EEG Data and affectiv data! "
//        //          << "Press any key to stop logging...\n" << std::endl;
//        //ofs.open("EEG_Data.csv",std::ios::trunc);
//		//ofs << header << std::endl;
//        //ofs2.open("Affectiv_Data.csv",std::ios::trunc);
//		//ofs2 << affectivSuitesName << std::endl;
//		
//		hData = EE_DataCreate();
//		EE_DataSetBufferSizeInSec(secs);
//
//		//std::cout << "Buffer size in secs:" << secs << std::endl;
//		printf ("--[Epoc Inited]-- \n");
//		return 0;
//	}
//
//	int Terminate()
//	{
//		//ofs.close();
//		//ofs2.close();
//		EE_DataFree(hData);
//    
//		EE_EngineDisconnect();
//		EE_EmoStateFree(eState);
//		EE_EmoEngineEventFree(eEvent);
//	}
//
//	int runEpoc()
//	{
//		// measure the real-time FPS
//		int FPSCount = 0;
//		clock_t t, dt;
//		t = clock(); // init the time 
//
//		while(1)
//		{
//			state = EE_EngineGetNextEvent(eEvent);
//			EE_Event_t eventType;
//
//			if (state == EDK_OK) 
//			{
//				eventType = EE_EmoEngineEventGetType(eEvent);
//				EE_EmoEngineEventGetUserId(eEvent, &userID);
//				EE_EmoEngineEventGetEmoState(eEvent, eState);
//
//				// Log the EmoState if it has been updated
//				if (eventType == EE_UserAdded) {
//					//std::cout << "User added";
//					EE_DataAcquisitionEnable(userID,true);
//					readytocollect = true;
//				}
//			
//				if (readytocollect && (eventType == EE_EmoStateUpdated)) {
//					unsigned int nSamplesTaken=0;
//
//					EE_DataUpdateHandle(0, hData);
//					EE_DataGetNumberOfSample(hData,&nSamplesTaken);
//
//					std::cout << "Epoc updated " << nSamplesTaken << std::endl;
//
//					if (nSamplesTaken != 0  ) 
//					{
//						double* data = new double[nSamplesTaken];
//						for (int sampleIdx=0 ; sampleIdx<(int)nSamplesTaken ; ++ sampleIdx) 
//						{
//							//epocMutex.lock();
//							for (int i = 0 ; i<sizeof(targetChannelList)/sizeof(EE_DataChannel_t) ; i++) 
//							{
//								// sizeof(targetChannelList)/sizeof(EE_DataChannel_t) = 22
//								EE_DataGet(hData, targetChannelList[i], data, nSamplesTaken);
//								//ofs << data[sampleIdx] << ",";
//								_epocpacket.EEG [i] = data[sampleIdx];
//							}
//							//epocMutex.unlock();
//							//ofs << std::endl;
//						}
//						delete[] data;
//					}
//
//					
//					affEngegement = ES_AffectivGetEngagementBoredomScore(eState);
//					affFrus = ES_AffectivGetFrustrationScore(eState);
//					affMed = ES_AffectivGetMeditationScore(eState);
//					affExcitement = ES_AffectivGetExcitementShortTermScore(eState);
//					affValence = ES_AffectivGetValenceScore(eState);
//					
//					//epocMutex.lock();
//					_epocpacket.EmoState[0] = affEngegement;
//					_epocpacket.EmoState[1] = affFrus;
//					_epocpacket.EmoState[2] = affMed;
//					_epocpacket.EmoState[3] = affExcitement;
//					_epocpacket.EmoState[4] = affValence;
//					//epocMutex.unlock();
//
//					// measure actual FPS
//					//FPSCount++;
//					//if (FPSCount == 20)
//					//{
//					//	FPSCount = 0;
//					//	dt = clock() - t; // get time difference
//					//	printf ("Actual FPS for EPOC: %f\n", 20.0/(((float)dt)/CLOCKS_PER_SEC));
//					//	t = clock(); // update the current clock
//					//}
//
//				}
//			}
//			// Sleep(100); // we can change this number to get different freq of data
//		}
//	}
//};
//
//
