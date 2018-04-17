// system related includes
#include <sstream>
#include <string>
#include <fstream>
#include "boost/thread.hpp"

// For Multimodal
#include "Multimodal.h"

int main()
{
	//***************************************//
	//           Init Multimodal             //
	//***************************************//
	Multimodal mm;
	// init(myoEnable, faceEnable, epocEnable)
	if (mm.init(1, 1, 1) == -1){
		exit(-1);
	}

	// FPS control
	float targetHz = 20;
	float T = 1.0/targetHz;
	clock_t t;
	float dt;
	t = clock();
	while(1)
	{
		//mm.Print();
		if (mm.WriteToTxt() == -1)
			exit(-1);

		// FPS control. wait until the T period has passed
		while (true){
			dt = float(clock() - t)/CLOCKS_PER_SEC; // get time difference (in seconds)
			if (dt < T)
				continue;
			else{
				t = clock(); // reset the reference time
				break; // get to the next loop
			}
		}

	}
	
	return 1;
}