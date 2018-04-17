#include "SocketServer.h"
#include "KinectBase.h"
#include <boost/thread.hpp>


int main()
{
	// get return value
	HRESULT hResult;

	// init kinect
	KinectBase k;
	hResult = k.InitSensor();
	if( FAILED( hResult ) ){
		std::cerr << "Error : Cannot init sensor" << std::endl;
		return -1;
	}

	// init color
	hResult = k.InitColor();
	if (FAILED(hResult)){
		std::cerr << "Error: Cannot init color" << std::endl;
	}

	// init depth
	hResult = k.InitDepth();
	if (FAILED(hResult)){
		std::cerr << "Error: Cannot init depth" << std::endl;
	}

	// set the depth range of interest
	hResult = k.SetDepthRange(800, 1000);
	if (FAILED(hResult)){
		std::cerr << "Error: Cannot set depth range" << std::endl;
	}

	// the forever updating thread
	//boost::thread* updateColorThread = new boost::thread(&KinectBase::UpdateColorForever, &k);
	//boost::thread* updateDepthThread = new boost::thread(&KinectBase::UpdateDepthForever, &k);
	//boost::thread* updateBodyThread = new boost::thread(&KinectBase::UpdateBodyForever, &k); // probamatic
	
	// init body (and face included)
	hResult = k.InitBody();
	if (FAILED(hResult)){
		std::cerr << "Error: Cannot init depth" << std::endl;
	}

	// init audio beam
	hResult = k.InitAudioBeam();
	if (FAILED(hResult)){
		std::cerr << "Error: Cannot init depth" << std::endl;
	}

	// init parameter for the FPS
	int FPSCount = 0;
	clock_t t, dt;
	t = clock(); // init the time 

	// init socket
	SocketServer socket;
	socket.InitSocket();
	while(1){
		//k.UpdateColor();
		//k.ShowColor();
		k.UpdateDepth();
		//k.ShowDepth();
		k.UpdateBody();
		k.ShowBody();
		k.UpdateAudioBeam();
		k.pac.WriteToBuffer();
		
		// here, we just send as fast as possible!!! No FPS control is needed.

		socket.Send(k.pac.buf);
		////// printf ("%s \n\n", k.pac.buf);

		// measure FPS
		// Debug: about 55 Hz when nobody, and 15 Hz when one person is present
		// Release: about 65Hz when nody, about 60Hz when one person is present
		//FPSCount++;
		//if (FPSCount == 20)
		//{
		//	FPSCount = 0;
		//	dt = clock() - t; // get time difference
		//	printf ("Actual FPS: %f\n", 20.0/(((float)dt)/CLOCKS_PER_SEC));
		//	t = clock(); // update the current clock
		//}

	}
}