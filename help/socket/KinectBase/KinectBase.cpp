#include "KinectBase.h"

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	SYSTEMTIME st;

	//GetSystemTime(&st);
	//cout << SystemTime.wHour << endl;
	GetLocalTime(&st);

	char buffer[256];
	sprintf(buffer,
		"%d_%02d_%02d_%02d_%02d_%02d_%03d",
		st.wYear,
		st.wMonth,
		st.wDay,
		st.wHour,
		st.wMinute,
		st.wSecond,
		st.wMilliseconds);

	std::string str(buffer);
	return str;
}

KinectBase::KinectBase()
{
	// opencv
	cv::setUseOptimized( true );
}

KinectBase::~KinectBase()
{
	KillBody();
	KillSensor();
}

#pragma region "Sensor"
int KinectBase::InitSensor()
{
	// Sensor
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// init the log name
	bool defaultFlag = false;
	if (defaultFlag == false){
		std::cout << "Please enter Kinect video name: ";
		std::getline(std::cin, id);
		std::cout << "You entered: " << id << std::endl;
	}
	else
	{
		id = "default";
	}

	//colorFilename = "Videos/" + filename + "_color" + ".avi";
	//depthFilename = "Videos/" + filename + "_depth" + ".avi";
	//bodyFilename = "Videos/" + id + "_body" + ".avi";
	bodyFilename = "Videos/" + id + ".avi";
	return 0;
}

int KinectBase::KillSensor()
{
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();
	exit(1);
	return 1;
}

#pragma endregion "Sensor"

#pragma region "Color"

int KinectBase::InitColor()
{
	// Source
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	hResult = pColorSource->get_FrameDescription( &pColorDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	pColorDescription->get_Width( &colorWidth ); // 1920
	pColorDescription->get_Height( &colorHeight ); // 1080
	colorBufferSize = colorWidth * colorHeight * 4 * sizeof( unsigned char );

	colorBufferMat = cv::Mat ( colorHeight, colorWidth, CV_8UC4 );
	colorMat = cv::Mat( colorHeight / 2, colorWidth / 2, CV_8UC4 );
	//colorWriter.open(colorFilename, CV_FOURCC('D','I','V','X'), 30, cv::Size(colorWidth/2, colorHeight/2), true);

	return 0;
}

int KinectBase::UpdateColorForever()
{
	while( 1 )
		KinectBase::UpdateColor();
	return 0;
}

int KinectBase::UpdateColor()
{
	colorMutex.lock();
	// Frame
	pColorFrame = nullptr;
	hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
	if( SUCCEEDED( hResult ) ){
		hResult = pColorFrame->CopyConvertedFrameDataToArray( colorBufferSize, reinterpret_cast<BYTE*>( colorBufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
		if( SUCCEEDED( hResult ) ){			
			cv::resize( colorBufferMat, colorMat, cv::Size(), 0.5, 0.5 );
			//colorWriter.write(colorMat);
		}
	}
	SafeRelease( pColorFrame );
	colorMutex.unlock();
	return 0;
}

cv::Mat KinectBase::GetColor()
{
	return colorMat;
}

int KinectBase::ShowColor()
{
	colorMutex.lock();
	cv::imshow( "Color", colorMat );
	if( cv::waitKey( 30 ) == VK_ESCAPE ){
		colorMutex.unlock();
		KillColor();
		return -1;
	}
	colorMutex.unlock();
	return 0;
}

int KinectBase::KillColor()
{
	cv::destroyWindow("Color");
	//colorWriter.release();
	SafeRelease( pColorSource );
	SafeRelease( pColorReader );
	SafeRelease( pColorDescription );
	exit(1);
	return 1;
}

#pragma endregion "Color"

#pragma region "Depth"
int KinectBase::InitDepth()
{
	// Source
	hResult = pSensor->get_DepthFrameSource( &pDepthSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	hResult = pDepthSource->OpenReader( &pDepthReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	pDepthDescription->get_Width( &depthWidth ); // 512
	pDepthDescription->get_Height( &depthHeight ); // 424
	depthBufferSize = depthWidth * depthHeight * sizeof( unsigned short );

	// Range ( Range of Depth is 500-8000[mm], Range of Detection is 500-4500[mm] ) 
	unsigned short min = 0;
	unsigned short max = 0;
	pDepthSource->get_DepthMinReliableDistance( &min ); // 500
	pDepthSource->get_DepthMaxReliableDistance( &max ); // 4500
	// std::cout << "Depth range : " << min << " - " << max << std::endl;
	
	depthBufferMat = cv::Mat( depthHeight, depthWidth, CV_16UC1 );
	depthMat = cv::Mat( depthHeight, depthWidth, CV_8UC1 );
	
	depthMinROI = 500;
	depthMaxROI = 8000;

	return 0;
}

int KinectBase::SetDepthRange(UINT16 _depthMinROI, UINT16 _depthMaxROI)
{
	// unit in mm. we normally work in the range of 800
	if (_depthMinROI > _depthMaxROI)
		return -1;
	if (_depthMinROI < 0 || _depthMaxROI < 0)
		return -1;
	depthMinROI = MAX(_depthMinROI, 500);
	depthMaxROI = MIN(_depthMaxROI, 8000);
	return 0;
}

int KinectBase::UpdateDepthForever()
{
	while(1)
		KinectBase::UpdateDepth();
	return 0;
}

int KinectBase::UpdateDepth()
{
	depthMutex.lock();
	// Frame
	pDepthFrame = nullptr;
	hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	if( SUCCEEDED( hResult ) ){	
		hResult = pDepthFrame->AccessUnderlyingBuffer( &depthBufferSize, reinterpret_cast<UINT16**>( &depthBufferMat.data ) );
		if( SUCCEEDED( hResult )){	

			// withour ROI depth, dark (0) is far, light (255) is near. I don't like it
			// depthBufferMat.convertTo( depthMat, CV_8U, -255.0f / 8000.0f, 255.0f );

			// with ROI depth, y = (x-min) * (255.0/(max-min)). 
			// put y in CV_8UC1 matrix, which will take care of overflow and underflow automatically
			// dark (0) is near, light (255) is far.

			cv::Mat temp = depthBufferMat - depthMinROI;
			temp.convertTo(depthMat, CV_8UC1, 255.0f/(depthMaxROI-depthMinROI));
			temp.release();
		}
	}
	SafeRelease( pDepthFrame );
	depthMutex.unlock();
	return 0;
}

cv::Mat KinectBase::GetDepth()
{
	return depthMat;
}

int KinectBase::ShowDepth()
{
	depthMutex.lock();
	cv::imshow( "Depth", depthMat );
	if( cv::waitKey( 30 ) == VK_ESCAPE ){
		depthMutex.unlock();
		KillDepth();
		return -1;
	}
	depthMutex.unlock();
	return 0;
}

int KinectBase::KillDepth()
{
	cv::destroyWindow("Depth");
	SafeRelease( pDepthSource );
	SafeRelease( pDepthReader );
	SafeRelease( pDepthDescription );
	exit(1);
	return 0;
}

#pragma endregion "Depth"

#pragma region "Mapper"
int KinectBase::InitCoordinateMapper()
{
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}
	return 0;
}
#pragma endregion "Mapper"

#pragma region "Body"

int KinectBase::InitBody(){
	// Source
	hResult = pSensor->get_ColorFrameSource( &pColorSource4Body );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	hResult = pSensor->get_BodyFrameSource( &pBodySource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	hResult = pColorSource4Body->OpenReader( &pColorReader4Body );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	hResult = pColorSource4Body->get_FrameDescription( &pBodyDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	pBodyDescription->get_Width( &width4Body ); // 1920
	pBodyDescription->get_Height( &height4Body ); // 1080
	bodyBufferSize = width4Body * height4Body * 4 * sizeof( unsigned char );

	bodyBufferMat = cv::Mat( height4Body, width4Body, CV_8UC4 );
	bodyMat = cv::Mat( height4Body / 2, width4Body / 2, CV_8UC4 );
	cv::namedWindow( "Body" );


	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255,   0,   0 );
	color[1] = cv::Vec3b(   0, 255,   0 );
	color[2] = cv::Vec3b(   0,   0, 255 );
	color[3] = cv::Vec3b( 255, 255,   0 );
	color[4] = cv::Vec3b( 255,   0, 255 );
	color[5] = cv::Vec3b(   0, 255, 255 );

	// Coordinate Mapper
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper4Body );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	// for the face
	facefeatures = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
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

	// init face feature calculation
	for( int count = 0; count < BODY_COUNT; count++ ){
		// face Source
		hResult = CreateFaceFrameSource( pSensor, 0, facefeatures, &pFaceSource[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateFaceFrameSource" << std::endl;
			return -1;
		}

		// faceReader
		hResult = pFaceSource[count]->OpenReader( &pFaceReader[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IFaceFrameSource::OpenReader()" << std::endl;
			return -1;
		}
	}

	// Face Property Table
	property[0] = "Happy";
	property[1] = "Engaged";
	property[2] = "WearingGlasses";
	property[3] = "LeftEyeClosed";
	property[4] = "RightEyeClosed";
	property[5] = "MouthOpen";
	property[6] = "MouthMoved";
	property[7] = "LookingAway";

	/// init mat
	//faceBufferMat = cv::Mat( height4Body, width4Body, CV_8UC4 );
	//faceMat = cv::Mat( height4Body / 2, width4Body / 2, CV_8UC4 );
	//cv::namedWindow( "Face" );

	// init video writer
	bodyWriter.open(bodyFilename, CV_FOURCC('D', 'I', 'V', 'X'), 20, cv::Size(int(colorWidth), int(colorHeight)), true);
	return 0;

}

int KinectBase::UpdateBody(){
	//bodyMutex.lock();

	// only allow one person
	bool onlyone = false;

	// Frame
	hResult = pColorReader4Body->AcquireLatestFrame( &pColorFrame4Body );
	if( SUCCEEDED( hResult ) ){
		hResult = pColorFrame4Body->CopyConvertedFrameDataToArray( bodyBufferSize, reinterpret_cast<BYTE*>( bodyBufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
		//bodyBufferMat.copyTo(faceBufferMat);
		if( SUCCEEDED( hResult ) ){
			cv::resize( bodyBufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
			//cv::resize( faceBufferMat, faceMat, cv::Size(), 0.5, 0.5 );
		}	
	}

	hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
	if( SUCCEEDED( hResult ) ){
		IBody* pBody[BODY_COUNT] = { 0 };
		hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );
		if( SUCCEEDED( hResult ) ){
			for( int count = 0; count < BODY_COUNT; count++ )
			{
				BOOLEAN bTracked = false;
				hResult = pBody[count]->get_IsTracked( &bTracked );
				if( SUCCEEDED( hResult ) && bTracked )
				{
					// Set TrackingID to Detect Face
					UINT64 trackingId = _UI64_MAX;
					hResult = pBody[count]->get_TrackingId( &trackingId );
					if( SUCCEEDED( hResult ) ){
						pFaceSource[count]->put_TrackingId( trackingId );
					}

					Joint joint[JointType::JointType_Count];
					hResult = pBody[ count ]->GetJoints( JointType::JointType_Count, joint );
					if( SUCCEEDED( hResult ) )
					{
						// Left Hand State
						HandState leftHandState = HandState::HandState_Unknown;
						hResult = pBody[count]->get_HandLeftState( &leftHandState );
						if( SUCCEEDED( hResult ) ){
							ColorSpacePoint colorSpacePoint = { 0 };
							hResult = pCoordinateMapper4Body->MapCameraPointToColorSpace( joint[JointType::JointType_HandLeft].Position, &colorSpacePoint );
							if( SUCCEEDED( hResult ) ){
								int x = static_cast<int>( colorSpacePoint.X );
								int y = static_cast<int>( colorSpacePoint.Y );
								if( ( x >= 0 ) && ( x < width4Body ) && ( y >= 0 ) && ( y < height4Body ) ){
									if( leftHandState == HandState::HandState_Open ){
										cv::circle( bodyBufferMat, cv::Point( x, y ), 45, cv::Scalar( 0, 128, 0 ), 5, CV_AA );
									}
									else if( leftHandState == HandState::HandState_Closed ){
										cv::circle( bodyBufferMat, cv::Point( x, y ), 45, cv::Scalar( 0, 0, 128 ), 5, CV_AA );
									}
									else if( leftHandState == HandState::HandState_Lasso ){
										cv::circle( bodyBufferMat, cv::Point( x, y ), 45, cv::Scalar( 128, 128, 0 ), 5, CV_AA );
									}
								}
							}
						}

						// Right Hand State
						HandState rightHandState = HandState::HandState_Unknown;
						hResult = pBody[count]->get_HandRightState( &rightHandState );
						if( SUCCEEDED( hResult ) ){
							
							ColorSpacePoint colorSpacePoint = { 0 };
							hResult = pCoordinateMapper4Body->MapCameraPointToColorSpace( joint[JointType::JointType_HandRight].Position, &colorSpacePoint );
							if( SUCCEEDED( hResult ) ){
								int x = static_cast<int>( colorSpacePoint.X );
								int y = static_cast<int>( colorSpacePoint.Y );
								if( ( x >= 0 ) && ( x < width4Body ) && ( y >= 0 ) && ( y < height4Body ) ){
									if( rightHandState == HandState::HandState_Open ){
										cv::circle( bodyBufferMat, cv::Point( x, y ), 45, cv::Scalar( 0, 128, 0 ), 5, CV_AA );
									}
									else if( rightHandState == HandState::HandState_Closed ){
										cv::circle( bodyBufferMat, cv::Point( x, y ), 45, cv::Scalar( 0, 0, 128 ), 5, CV_AA );
									}
									else if( rightHandState == HandState::HandState_Lasso ){
										cv::circle( bodyBufferMat, cv::Point( x, y ), 45, cv::Scalar( 128, 128, 0 ), 5, CV_AA );
									}
								}
							}
						}

						// Joint
						//for( int type = 0; type < JointType::JointType_Count; type++ ){
						//	//if (13 <= type <= 15 || 17 <= type <= 19) // skip the 3 joints on left leg and right leg
						//	//	continue;
						//	ColorSpacePoint colorSpacePoint = { 0 };
						//	pCoordinateMapper4Body->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
						//	int x = static_cast<int>( colorSpacePoint.X );
						//	int y = static_cast<int>( colorSpacePoint.Y );
						//	if( ( x >= 0 ) && ( x < width4Body ) && ( y >= 0 ) && ( y < height4Body ) ){
						//		//cv::circle( bodyBufferMat, cv::Point( x, y ), 5, static_cast< cv::Scalar >( color[count] ), -1, CV_AA );
						//		cv::circle( bodyBufferMat, cv::Point( x, y ), 10, cv::Scalar(255,0,0), -1, CV_AA );
						//	}
						//}

						// Draw the skeleton of body (bones)
						ColorSpacePoint* colorSpacePosition = new ColorSpacePoint[_countof(joint)];
						for (int j = 0; j < _countof(joint); ++j)
							pCoordinateMapper4Body->MapCameraPointToColorSpace(joint[j].Position, &colorSpacePosition[j]);
						
						DrawBody(joint, colorSpacePosition);

						// Lean
						PointF lean;
						hResult = pBody[count]->get_Lean( &lean );
						if( SUCCEEDED( hResult ) ){
							;
							//std::cout << "lean : " << lean.X << ", " << lean.Y << std::endl;
						}

						// fill the buffer
						if (onlyone == false){
							onlyone = true;
							pac.leftHandState = leftHandState;
							pac.rightHandState = rightHandState;
							for( int type = 0; type < JointType::JointType_Count; type++ ){
								pac.joint[type].Position = joint[type].Position;
							}
							pac.lean = lean;
						}
					}
				}
			}
			//cv::resize( bodyBufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
		}
		for( int count = 0; count < BODY_COUNT; count++ ){
			SafeRelease( pBody[count] );
		}
	}
	SafeRelease( pBodyFrame );

	// Face Frame
	// only allow one person
	onlyone = false;

	// std::system( "cls" );
	for( int count = 0; count < BODY_COUNT; count++ )
	{
		IFaceFrame* pFaceFrame = nullptr;
		hResult = pFaceReader[count]->AcquireLatestFrame( &pFaceFrame );
		if( SUCCEEDED( hResult ) && pFaceFrame != nullptr ){
			BOOLEAN bFaceTracked = false;
			hResult = pFaceFrame->get_IsTrackingIdValid( &bFaceTracked );
			if( SUCCEEDED( hResult ) && bFaceTracked ){
				IFaceFrameResult* pFaceResult = nullptr;
				hResult = pFaceFrame->get_FaceFrameResult( &pFaceResult );
				if( SUCCEEDED( hResult ) && pFaceResult != nullptr ){
					
					std::vector<std::string> result;

					// Face Point
					PointF facePoint[FacePointType::FacePointType_Count];
					hResult = pFaceResult->GetFacePointsInColorSpace( FacePointType::FacePointType_Count, facePoint );
					if( SUCCEEDED( hResult ) ){
						cv::circle( bodyBufferMat, cv::Point( static_cast<int>( facePoint[0].X ), static_cast<int>( facePoint[0].Y ) ), 2, cv::Scalar(0,255,0), -1, CV_AA ); // Eye (Left)
						cv::circle( bodyBufferMat, cv::Point( static_cast<int>( facePoint[1].X ), static_cast<int>( facePoint[1].Y ) ), 2, cv::Scalar(0,255,0), -1, CV_AA ); // Eye (Right)
						cv::circle( bodyBufferMat, cv::Point( static_cast<int>( facePoint[2].X ), static_cast<int>( facePoint[2].Y ) ), 2, cv::Scalar(0,255,0), -1, CV_AA ); // Nose
						cv::circle( bodyBufferMat, cv::Point( static_cast<int>( facePoint[3].X ), static_cast<int>( facePoint[3].Y ) ), 2, cv::Scalar(0,255,0), -1, CV_AA ); // Mouth (Left)
						cv::circle( bodyBufferMat, cv::Point( static_cast<int>( facePoint[4].X ), static_cast<int>( facePoint[4].Y ) ), 2, cv::Scalar(0,255,0), -1, CV_AA ); // Mouth (Right)
					}

					// Face Bounding Box
					RectI boundingBox;
					hResult = pFaceResult->get_FaceBoundingBoxInColorSpace( &boundingBox );
					if( SUCCEEDED( hResult ) ){
						cv::rectangle( bodyBufferMat, cv::Rect( boundingBox.Left, boundingBox.Top, boundingBox.Right - boundingBox.Left, boundingBox.Bottom - boundingBox.Top ), cv::Scalar(0,255,0), 2);
					}

					// Face Rotation
					Vector4 faceRotation;
					hResult = pFaceResult->get_FaceRotationQuaternion( &faceRotation );
					int pitch, yaw, roll;
					if( SUCCEEDED( hResult ) ){
						ExtractFaceRotationInDegrees( &faceRotation, &pitch, &yaw, &roll );
						result.push_back( "Pitch, Yaw, Roll : " + std::to_string( pitch ) + ", " + std::to_string( yaw ) + ", " + std::to_string( roll ) );
					}

					// Face Property
					DetectionResult faceProperty[FaceProperty::FaceProperty_Count];
					hResult = pFaceResult->GetFaceProperties( FaceProperty::FaceProperty_Count, faceProperty );
					if( SUCCEEDED( hResult ) ){
						for( int count = 0; count < FaceProperty::FaceProperty_Count; count++ ){
							switch( faceProperty[count] ){
								case DetectionResult::DetectionResult_Unknown:
									result.push_back( property[count] + " : Unknown" );
									break;
								case DetectionResult::DetectionResult_Yes:
									result.push_back( property[count] + " : Yes" );
									break;
								case DetectionResult::DetectionResult_No:
									result.push_back( property[count] + " : No" );
									break;
								case DetectionResult::DetectionResult_Maybe:
									result.push_back( property[count] + " : Mayby" );
									break;
								default:
									break;
							}
						}
					}

					// put text for face property
					/*if( boundingBox.Left && boundingBox.Bottom ){
						int offset = 30;
						for( std::vector<std::string>::iterator it = result.begin(); it != result.end(); it++, offset += 30 ){
							cv::putText( bodyBufferMat, *it, cv::Point( boundingBox.Left, boundingBox.Bottom + offset ), cv::FONT_HERSHEY_COMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
						}
					}*/

					// fill the buffer
					if (onlyone == false){
						onlyone = true;
						for (int8_t i = 0; i < FacePointType::FacePointType_Count; i ++)
							pac.facePoint[i] = facePoint[i];
						pac.boundingBox = boundingBox;
						pac.roll = roll;
						pac.pitch = pitch;
						pac.yaw = yaw;
						for (int8_t i = 0; i < FaceProperty::FaceProperty_Count; i ++)
							pac.faceProperty[i] = faceProperty[i];
					}
				}
				SafeRelease( pFaceResult );
			}
		}
		SafeRelease( pFaceFrame );
	}
	cv::resize( bodyBufferMat, bodyMat, cv::Size(), 0.5, 0.5 );

	SafeRelease( pColorFrame4Body );
	
	//bodyMutex.unlock();
	return 0;
}

void KinectBase::DrawBody(const Joint* pJoints, const ColorSpacePoint* colorSpacePosition)
{
	//---------------------------body-------------------------------
	DrawBone(pJoints, colorSpacePosition, JointType_Head, JointType_Neck);
	DrawBone(pJoints, colorSpacePosition, JointType_Neck, JointType_SpineShoulder);
	DrawBone(pJoints, colorSpacePosition, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(pJoints, colorSpacePosition, JointType_SpineMid, JointType_SpineBase);
	DrawBone(pJoints, colorSpacePosition, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(pJoints, colorSpacePosition, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(pJoints, colorSpacePosition, JointType_SpineBase, JointType_HipRight);
	DrawBone(pJoints, colorSpacePosition, JointType_SpineBase, JointType_HipLeft);

	// -----------------------Right Arm ------------------------------------ 
	DrawBone(pJoints, colorSpacePosition, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(pJoints, colorSpacePosition, JointType_ElbowRight, JointType_WristRight);
	DrawBone(pJoints, colorSpacePosition, JointType_WristRight, JointType_HandRight);
	DrawBone(pJoints, colorSpacePosition, JointType_HandRight, JointType_HandTipRight);
	DrawBone(pJoints, colorSpacePosition, JointType_WristRight, JointType_ThumbRight);

	//----------------------------------- Left Arm--------------------------
	DrawBone(pJoints, colorSpacePosition, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(pJoints, colorSpacePosition, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(pJoints, colorSpacePosition, JointType_WristLeft, JointType_HandLeft);
	DrawBone(pJoints, colorSpacePosition, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(pJoints, colorSpacePosition, JointType_WristLeft, JointType_ThumbLeft);

	// ----------------------------------Right Leg--------------------------------
	//DrawBone(pJoints, colorSpacePosition, JointType_HipRight, JointType_KneeRight);
	//DrawBone(pJoints, colorSpacePosition, JointType_KneeRight, JointType_AnkleRight);
	//DrawBone(pJoints, colorSpacePosition, JointType_AnkleRight, JointType_FootRight);

	// -----------------------------------Left Leg---------------------------------
	//DrawBone(pJoints, colorSpacePosition, JointType_HipLeft, JointType_KneeLeft);
	//DrawBone(pJoints, colorSpacePosition, JointType_KneeLeft, JointType_AnkleLeft);
	//DrawBone(pJoints, colorSpacePosition, JointType_AnkleLeft, JointType_FootLeft);

	/*CvScalar color = cv::Scalar(0, 0, 255);
	for (int i = 0; i < JointType_Count; ++i)
	{
		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			circle(D, cvPoint(colorSpacePosition[i].X, colorSpacePosition[i].Y), 5, color, -1);
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			circle(D, cvPoint(colorSpacePosition[i].X, colorSpacePosition[i].Y), 5, color, -1);
		}	
	}*/

}

void KinectBase::DrawBone(const Joint* pJoints, const ColorSpacePoint* colorSpacePosition, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	CvPoint p1 = cvPoint(colorSpacePosition[joint0].X, colorSpacePosition[joint0].Y),
			p2 = cvPoint(colorSpacePosition[joint1].X, colorSpacePosition[joint1].Y);

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		// white means good
		cv::line(bodyBufferMat, p1, p2, cvScalar(255, 255, 255), 10);
	}
	else
	{
		// red means not too good. One tracked and one inferred
		cv::line(bodyBufferMat, p1, p2, cvScalar(0, 0, 255), 10);
	}
}


int KinectBase::UpdateBodyForever(){
	while(1)
		UpdateBody();
	return 0;
}

int KinectBase::ShowBody(){
	//bodyMutex.lock();
	//cv::imshow( "Body", bodyMat );
	//cv::imshow( "Face", faceMat );

	// prepare bodyMatBGR, write time as well
	cv::cvtColor(bodyMat, bodyMatBGR, CV_BGRA2BGR);
	cv::flip(bodyMatBGR, bodyMatBGR, 1); // flip image left right

	std::string currentTime = currentDateTime();
	//cv::putText(bodyMatBGR, id+"_"+currentTime, cvPoint(30, 50), cv::FONT_HERSHEY_COMPLEX, 1, cvScalar(255, 0, 0), 1.5, CV_AA);
	cv::resize(bodyMatBGR, bodyMatBGR, cv::Size(), 2, 2);
	cv::imshow( "Body", bodyMatBGR );
	bodyWriter.write(bodyMatBGR);

	if (cv::waitKey( 10 ) == VK_ESCAPE){
		//bodyMutex.unlock();
		KillBody();
		exit(1);
	}
	//bodyMutex.unlock();
	return 1;
}

int KinectBase::KillBody(){
	cv::destroyWindow("Body");
	cv::destroyWindow("Face");
	bodyWriter.release();
	//bodyMutex.unlock();
	SafeRelease( pColorSource4Body );
	SafeRelease( pBodySource );
	SafeRelease( pColorReader4Body );
	SafeRelease( pBodyReader );
	SafeRelease( pBodyDescription );
	SafeRelease( pCoordinateMapper4Body );
	for( int count = 0; count < BODY_COUNT; count++ ){
		SafeRelease( pFaceSource[count] );
		SafeRelease( pFaceReader[count] );
	}
	return 0;
}

cv::Mat KinectBase::GetBodyMat(){
	return bodyMat;
}

// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
inline void ExtractFaceRotationInDegrees( const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll )
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees
	*pPitch = static_cast<int>( std::atan2( 2 * ( y * z + w * x ), w * w - x * x - y * y + z * z ) / M_PI * 180.0f );
	*pYaw = static_cast<int>( std::asin( 2 * ( w * y - x * z ) ) / M_PI * 180.0f );
	*pRoll = static_cast<int>( std::atan2( 2 * ( x * y + w * z ), w * w + x * x - y * y - z * z ) / M_PI * 180.0f );
}

#pragma endregion "Body"

#pragma region "AudioBeam"

int KinectBase::InitAudioBeam(){
	// Source
	hResult = pSensor->get_AudioSource( &pAudioSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_AudioSource()" << std::endl;
		return -1;
	}

	// Reader
	hResult = pAudioSource->OpenReader( &pAudioReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IAudioSource::OpenReader()" << std::endl;
		return -1;
	}
	return 1;
}

int KinectBase::UpdateAudioBeam(){
	// only allow one audio
	bool onlyone = false;

	// Frame List
	IAudioBeamFrameList* pAudioFrameList = nullptr;
	hResult = pAudioReader->AcquireLatestBeamFrames( &pAudioFrameList );
	if( SUCCEEDED( hResult ) ){
		UINT count = 0;
		hResult = pAudioFrameList->get_BeamCount( &count );
		if( SUCCEEDED( hResult ) ){
			for( unsigned int index = 0; index < count; index++ ){
				// Frame
				IAudioBeamFrame* pAudioFrame = nullptr;
				hResult = pAudioFrameList->OpenAudioBeamFrame( index, &pAudioFrame );
				if( SUCCEEDED( hResult ) ){
					// Get Beam Angle and Confidence
					IAudioBeam* pAudioBeam = nullptr;
					hResult = pAudioFrame->get_AudioBeam( &pAudioBeam );
					if( SUCCEEDED( hResult ) ){
						FLOAT angle = 0.0f;
						FLOAT confidence = 0.0f;
						pAudioBeam->get_BeamAngle( &angle ); // radian [-0.872665f, 0.872665f]
						pAudioBeam->get_BeamAngleConfidence( &confidence ); // confidence [0.0f, 1.0f]

						// Convert from radian to degree : degree = radian * 180 / Pi
						if( confidence > 0.5f ){
							if (onlyone == false){
								pac.angle = angle;
								pac.confidence = confidence;
								onlyone = true;
							}
							//std::cout << "Audio beam Index : " << index << ", Angle : " << angle * 180.0f / M_PI << ", Confidence : " << confidence << std::endl;
						}
						else{
							pac.angle = 0;
							pac.confidence = 0;
						}
					}
					SafeRelease( pAudioBeam );
				}
				SafeRelease( pAudioFrame );
			}
		}
	}
	SafeRelease( pAudioFrameList );

	// Input Key ( Exit ESC key )
	if( GetKeyState( VK_ESCAPE ) < 0 ){
		return -1;
	}
	return 1;
}

int KinectBase::ForeverUpdateAudioBeam(){
	while (1)
		UpdateAudioBeam();
	return 1;
}

int KinectBase::KillAudioBeam(){
	SafeRelease( pAudioSource );
	SafeRelease( pAudioReader );
	exit(1);
	return 1;
}

#pragma endregion "AudioBeam"

