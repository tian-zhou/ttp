#ifndef __MYO__
#define __MYO__

// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <stdlib.h> // for system("cls");
#include <time.h>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include "myo/myo.hpp"

// since we don't have math.h, we include the following line
#define M_PI       3.14159265358979323846

boost::mutex myoMutex;

// define the structure for the packet of myo
struct MYO_PACKET {
	float roll, pitch, yaw;
	std::array<int8_t, 8> emg;
	myo::Vector3 < float > accel;
	myo::Vector3 < float > gyro;
	float timestamp;
	myo::Arm whichArm;
	myo::XDirection xdir;
	bool onArm;
	int8_t rssi;
	bool isUnlocked;
	myo::Pose currentPose;


	void Print()
	{
		// Clear the current line, since I have multiple lines of printing, \r does not work
		// I choose to use cls to clear the whole screen
		// std::cout << '\r';	

		std::cout << "TimeStamp: " << timestamp << std::endl;

		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		//std::cout << "Orientation [roll] [pitch] [raw] range (1-18 stars):" << std::endl;
		//std::cout << '[' << std::string(roll, '*') << std::string(18 - roll, ' ') << ']'
		//	<< '[' << std::string(pitch, '*') << std::string(18 - pitch, ' ') << ']'
		//	<< '[' << std::string(yaw, '*') << std::string(18 - yaw, ' ') << ']';
		//std::cout << std::endl; 

		std::cout << "roll: " << roll << "  pitch: " << pitch << "  yaw: " << yaw << std::endl;
		

		// print out the Pose data.
		std::cout << "Pose [isUnlocked] [whichArm] [Pose]:" << std::endl;
		if (onArm) {
			// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

			// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
			// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
			// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
			std::string poseString = currentPose.toString();

			std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
				<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
				<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		} else {
			// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
			std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
		}
		std::cout << std::endl;

		// print out the EMG data.
		std::cout << "EMG array, size 8, range (-128 ~ 127):" << std::endl;;
		for (size_t i = 0; i < emg.size(); i++) {
			std::ostringstream oss;
			oss << static_cast<int>(emg[i]);
			std::string emgString = oss.str();

			std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
		}
		std::cout << std::endl;

		//std::cout << std::flush;
	}


	// ***** for Mayo armband ***** //
	// packet structure (each data takes one line, frame rate is 50Hz, set in the main function):
	// [timestamp][onArm][xdir][whichArm][isUnlocked][int(rssi)][currentPose][int(emgSamples) x8][roll][pitch][yaw][accel x3][gyro x3
	void WriteToTxt(std::ofstream &fw){
		char sep = ' ';
		myoMutex.lock();
		fw << timestamp << sep;
		fw << onArm << sep << xdir << sep << whichArm << sep << isUnlocked << sep << int(rssi) << sep << currentPose._type << sep;
		for (size_t i = 0; i < 8; i++) {
			// convert signed char to integer number to save
			fw << int(emg[i]) << sep;
		}
		fw << roll << sep << pitch << sep << yaw << sep;
		fw << accel[0] << sep << accel[1] << sep << accel[2] << sep;
		fw << gyro[0] << sep << gyro[1] << sep << gyro[2] << sep;
		myoMutex.unlock();
	}
};


// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
	// constructor
	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose(), emg()
	{
		initTimeFlag = false;
	}

	// destructors
	~DataCollector()
	{
		fwriter.close();
	}

	// --------------------------------------
	// Local Variable definition
	// -------------------------------------

	// keep track of the timestamp of the event, it is update in all the callback function
	// Timestamps are 64 bit unsigned integers that correspond to a number of microseconds 
	// since some (unspecified) period in time. Timestamps are monotonically non-decreasing.
	uint64_t timestamp;
	uint64_t timestamp0;
	bool initTimeFlag;

	// These values are set by onArmSync() and onArmUnsync().
	bool onArm;
	myo::XDirection xdir;
	myo::Arm whichArm;

	// This is set by onUnlocked() and onLocked().
	bool isUnlocked;

	// These values are set by onOrientationData() and onPose().
	int roll_w, pitch_w, yaw_w;
	float roll, pitch, yaw;
	myo::Pose currentPose;

	// The values of this array is set by onEmgData().
	std::array<int8_t, 8> emg;

	// This 3D vector holds 3 float numbers indicating the acceleration of the MYO in units of g
	// set by onAccelerometerData()
	myo::Vector3 < float > accel;

	// this 3D vector holds 3 float number, indicating the angular velocity in units of degree/s
	// set by onGyroscopeData()

	myo::Vector3 < float > gyro;

	//The RSSI (received signal strength indication) of myo, set by onRssi()
	int8_t rssi;

	// file writer to save data for further analysis
	std::ofstream fwriter;

	// --------------------------------------
	// virtual callback function overridden
	// -------------------------------------

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t _timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		timestamp = _timestamp;
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;
		emg.fill(0);
	}

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
	// the rate is 200Hz
	void onEmgData(myo::Myo* myo, uint64_t _timestamp, const int8_t* _emg)
	{

		if (initTimeFlag == false){
			initTimeFlag = true;
			timestamp0 = _timestamp;
		}

		timestamp = _timestamp;
		for (int i = 0; i < 8; i++) {
			emg[i] = _emg[i];
		}
	}

	// onAccelerometerData() is called when a paired Myo has provided new accelerometer data in units of g.
	void onAccelerometerData(myo::Myo* myo, uint64_t _timestamp, const myo::Vector3 <float> &_accel)
	{
		timestamp = _timestamp;
		accel = _accel;
	}

	// onGyroscopeData() is called when a paired Myo has provided new gyroscope data in units of degree/s
	void onGyroscopeData(myo::Myo *myo, uint64_t _timestamp, const myo::Vector3 <float> & _gyro)
	{
		timestamp = _timestamp;
		gyro = _gyro;
	}

	// Called when a paired Myo has provided a new RSSI value.
	void onRssi(myo::Myo *myo, uint64_t _timestamp, int8_t _rssi)
	{
		timestamp = _timestamp;
		rssi = _rssi;
	}



	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t _timestamp, const myo::Quaternion<float>& quat)
	{
		timestamp = _timestamp;
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		// Convert the floating point angles in radians to a scale from 0 to 18.
		roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
	}

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t _timestamp, myo::Pose pose)
	{
		timestamp = _timestamp;
		currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
			// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
			// Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);

			// Notify the Myo that the pose has resulted in an action, in this case changing
			// the text on the screen. The Myo will vibrate.
			myo->notifyUserAction();
			//myo->vibrate(myo::Myo::vibrationShort);
			//myo->vibrate(myo::Myo::vibrationMedium);
			//myo->vibrate(myo::Myo::vibrationLong);

		} else {
			// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
			// are being performed, but lock after inactivity.
			myo->unlock(myo::Myo::unlockTimed);
		}
	}

	// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmSync(myo::Myo* myo, uint64_t _timestamp, myo::Arm arm, myo::XDirection xDirection)
	{
		timestamp = _timestamp;
		onArm = true;
		whichArm = arm;
		xdir = xDirection;
	}

	// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
	// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
	// when Myo is moved around on the arm.
	void onArmUnsync(myo::Myo* myo, uint64_t _timestamp)
	{
		timestamp = _timestamp;
		onArm = false;
	}

	// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	void onUnlock(myo::Myo* myo, uint64_t _timestamp)
	{
		timestamp = _timestamp;
		isUnlocked = true;
	}

	// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	void onLock(myo::Myo* myo, uint64_t _timestamp)
	{
		timestamp = _timestamp;
		isUnlocked = false;
	}


	// --------------------------------------
	// auxiliary function definition
	// -------------------------------------

	// We define this function to print the current values that were updated by the on...() functions above.
	void print()
	{
		// Clear the current line, since I have multiple lines of printing, \r does not work
		// I choose to use cls to clear the whole screen
		system("cls");
		// std::cout << '\r';	

		std::cout << "TimeStamp: " << timestamp << std::endl;

		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		std::cout << "Orientation [roll] [pitch] [raw] range (1-18 stars):" << std::endl;
		std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
			<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
			<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
		std::cout << std::endl; 

		// print out the Pose data.
		std::cout << "Pose [isUnlocked] [whichArm] [Pose]:" << std::endl;
		if (onArm) {
			// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

			// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
			// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
			// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
			std::string poseString = currentPose.toString();

			std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
				<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
				<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		} else {
			// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
			std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
		}
		std::cout << std::endl;

		// print out the EMG data.
		std::cout << "EMG array, size 8, range (-128 ~ 127):" << std::endl;;
		for (size_t i = 0; i < emg.size(); i++) {
			std::ostringstream oss;
			oss << static_cast<int>(emg[i]);
			std::string emgString = oss.str();

			std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
		}
		std::cout << std::endl;

		//std::cout << std::flush;
	}

	// Init the fwrite
	void InitWriter()
	{
		bool newFileFlag = false;
		std::string logName;
		char* FullName = new char[50];
		if (newFileFlag)
		{
			// the user will enter a new log file name
			std::cout << "Please enter log file name: " << std::endl;
			std::cin >> logName;
			char* ch_logName = new char[40];
			strcpy(ch_logName, logName.c_str());
			strcat(ch_logName, ".txt");
			strcpy(FullName, "log/");
			strcat(FullName, ch_logName);
		}
		else
		{
			// the default log file name
			FullName = "log/defaultMyoLog.txt";
		}
		fwriter.open(FullName, std::ios::out); // |std::ios::app
		if (!fwriter.is_open())
			printf("!!!!Error!!!!! File writer cannot init... \n");
	}

	// write the raw signal into a local txt file for further analysis
	void write()
	{
		if (fwriter.is_open())
		{
			// write relevant data into the text file in the following format:
			// packet structure (each data takes one line, frame rate is 50Hz, set in the main function):
			// [timestamp][onArm][xdir][whichArm][isUnlocked][int(rssi)][currentPose][int(emgSamples) x8][roll][pitch][yaw][accel x3][gyro x3]

			fwriter << timestamp << ' ';
			fwriter << onArm << ' ' << xdir << ' ' << whichArm << ' ' << isUnlocked << ' ' << int(rssi) << ' ' << currentPose << ' ';
			for (size_t i = 0; i < emg.size(); i++) {
				// convert signed char to integer number to save
				fwriter << int(emg[i]) << ' ';
			}
			fwriter << roll_w << ' ' << pitch_w << ' ' << yaw_w << ' ';
			fwriter << accel[0] << ' ' << accel[1] << ' ' << accel[2] << ' ';
			fwriter << gyro[0] << ' ' << gyro[1] << ' ' << gyro[2] << ' ';
			fwriter << std::endl; 
		}
	}

	void FillPacket(MYO_PACKET &_myo_packet)
	{
		myoMutex.lock();
		_myo_packet.timestamp = (timestamp - timestamp0) * std::pow(10,-6);
		_myo_packet.accel = accel;
		_myo_packet.gyro = gyro/(180.0/M_PI); // convert from degree/s to radian/s
		_myo_packet.pitch = pitch;
		_myo_packet.roll = roll;
		_myo_packet.yaw = yaw;
		_myo_packet.emg = emg;
		_myo_packet.whichArm = whichArm;
		_myo_packet.onArm = onArm;
		_myo_packet.xdir = xdir;
		_myo_packet.currentPose = currentPose;
		_myo_packet.rssi = rssi;
		_myo_packet.isUnlocked = isUnlocked;
		_myo_packet.currentPose = currentPose;
		myoMutex.unlock();
	}

};

int run_myo(MYO_PACKET &myo_packet)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
			printf ("!![Myo failed]!!");
		}

		// We've found a Myo.
		std::cout << "--[Myo Inited]--" << std::endl << std::endl;

		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// init the file writer to save data
		// collector.InitWriter();

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		// Finally we enter our main loop.
		// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
		// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
		int Hz = 20;
		//printf ("Set FPS for myo: %i\n", Hz);

		while (1) {
			hub.run(1000 / Hz);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			// collector.print();

			// compose the packet
			collector.FillPacket(myo_packet);

			// write the data into a local text file
			//collector.write();
		}

		// If a standard exception occurred, we print out its message and exit.
	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}


#endif