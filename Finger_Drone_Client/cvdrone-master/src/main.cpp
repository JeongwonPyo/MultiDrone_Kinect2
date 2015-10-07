#define WIN32_LEAN_AND_MEAN

#include <WinSock2.h>
#include <WS2tcpip.h>
#include <fstream>

#define PORT		 27015
#define IP 			 "169.254.179.2"	// Erase it and write your server IP.
#define MAXLINE 1024


#pragma comment(lib, "Ws2_32.lib")

#include "ardrone/ardrone.h"

#include "kinect.h"



///////////////////	For Marker Include	////////////////////////////////////////
// Marker detector
#include ".\3rdparty\packtpub\MarkerDetector.hpp"

// Parameter for calibration pattern
#define PAT_ROWS   (7)                  // Rows of pattern
#define PAT_COLS   (10)                 // Columns of pattern
#define CHESS_SIZE (24.0)               // Size of a pattern [mm]

// Global variables
cv::Mat mapx, mapy;
CameraCalibration calibration;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef std::vector<cv::Point>    PointsVector;
typedef std::vector<PointsVector> ContoursVector;

cv::Mat g_camMatrix;
cv::Mat g_distCoeff;
std::vector<cv::Point3f> g_markerCorners3d;
std::vector<cv::Point2f> g_markerCorners2d;
cv::Size g_markerSize(200, 200);

#define MARKER_ID_1 213
#define MARKER_ID_2 85
#define MARKER_ID_3 149
#define MARKER_ID_4 197
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////


int LoadCalibrationInfo(CameraCalibration &dstCal, cv::Mat &dstx, cv::Mat &dsty, cv::Size size, std::string filename)
{
	// Open XML file
	cv::FileStorage rfs(filename, cv::FileStorage::READ);
	if (!rfs.isOpened()) {
		return -1;
	}

	// Load camera parameters
	cv::Mat cameraMatrix, distCoeffs;
	rfs["intrinsic"] >> cameraMatrix;
	rfs["distortion"] >> distCoeffs;

	// Create undistort map
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, size, CV_32FC1, dstx, dsty);

	// Set camera parameters
	float fx = cameraMatrix.at<double>(0, 0);
	float fy = cameraMatrix.at<double>(1, 1);
	float cx = cameraMatrix.at<double>(0, 2);
	float cy = cameraMatrix.at<double>(1, 2);
	//calibration = CameraCalibration(fx, fy, cx, cy);
	dstCal = CameraCalibration(fx, fy, size.width / 2, size.height / 2);

	cv::Mat(3, 3, CV_32F, const_cast<float*>(&calibration.getIntrinsic().data[0])).copyTo(g_camMatrix);
	cv::Mat(4, 1, CV_32F, const_cast<float*>(&calibration.getDistorsion().data[0])).copyTo(g_distCoeff);

	return 1;
}


// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int Auto_Calibration()
{
	// Open XML file
	std::string filename("camera.xml");

	std::fstream file(filename.c_str(), std::ios::in);

	//string aviname = format("calibration.avi");
	char* aviname = "calibration.avi";
	VideoCapture capture(aviname);
	Mat frame;

	if (!capture.isOpened())
		throw "Error when reading steam_avi";

	// Not found
	//if (!file.is_open()) {
	//	cout << " Fail to open xml" << endl;
	//}

	///*
	// Image buffer
	vector<Mat> images;
	//std::cout << "Press Space key to capture an image" << std::endl;
	//std::cout << "Press Esc to exit" << std::endl;

	// Main loop
	while (images.size() != 4) {


		capture >> frame;
		if (frame.empty())
			break;

		// Key iput
		int key = cv::waitKey(1);
		if (key == 0x1b) break;

		// Get an image
		//frame = ardrone.getImage();

		// Convert to grayscale
		cv::Mat gray;
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		// Detect a chessboard
		cv::Size size(PAT_COLS, PAT_ROWS);
		std::vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(gray, size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

		// Chessboard detected
		if (found) {
			// Draw it
			cv::drawChessboardCorners(frame, size, corners, found);

			// Space key was pressed
			//if (key == ' ') {
			// Add to buffer
			images.push_back(gray);
			//}
		}

		// Show the image
		std::ostringstream stream;
		stream << "Captured " << images.size() << " image(s).";
		cv::putText(frame, stream.str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		cv::imshow("Camera Calibration", frame);
	}

	// We have enough samples
	if (images.size() > 4) {
		cv::Size size(PAT_COLS, PAT_ROWS);
		std::vector<std::vector<cv::Point2f>> corners2D;
		std::vector<std::vector<cv::Point3f>> corners3D;

		for (size_t i = 0; i < images.size(); i++) {
			// Detect a chessboard
			std::vector<cv::Point2f> tmp_corners2D;
			bool found = cv::findChessboardCorners(images[i], size, tmp_corners2D);

			// Chessboard detected
			if (found) {
				// Convert the corners to sub-pixel
				cv::cornerSubPix(images[i], tmp_corners2D, cvSize(11, 11), cvSize(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
				corners2D.push_back(tmp_corners2D);

				// Set the 3D position of patterns
				const float squareSize = CHESS_SIZE;
				std::vector<cv::Point3f> tmp_corners3D;
				for (int j = 0; j < size.height; j++) {
					for (int k = 0; k < size.width; k++) {
						tmp_corners3D.push_back(cv::Point3f((float)(k*squareSize), (float)(j*squareSize), 0.0));
					}
				}
				corners3D.push_back(tmp_corners3D);
			}
		}

		// Estimate camera parameters
		cv::Mat cameraMatrix, distCoeffs;
		std::vector<cv::Mat> rvec, tvec;
		cv::calibrateCamera(corners3D, corners2D, images[0].size(), cameraMatrix, distCoeffs, rvec, tvec);
		std::cout << cameraMatrix << std::endl;
		std::cout << distCoeffs << std::endl;

		// Save them
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
		fs << "intrinsic" << cameraMatrix;
		fs << "distortion" << distCoeffs;

		fs.release();
	}

	// Destroy windows
	cv::destroyAllWindows();
	//*/

}

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	CBodyBasics kinect;

    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

	

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

	//Auto_Calibration();

	// Open XML file
	std::string filename("camera.xml");

	///////////////////////////////////////////////////////////////////////////////////////////////
	cv::Size imageSize(640, 360);
	int ret = LoadCalibrationInfo(calibration, mapx, mapy, imageSize, filename);
	if (ret != 1)
	{
		std::cout << "Failed to open the XML file" << std::endl;
		return -1;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////

	unsigned long start = 0, now, temp_Time;

	int landing_Flag_Left = 0, left_Flag_Left = 0, right_Flag_Left = 0, forward_Flag_Left = 0, backward_Flag_Left = 0, left_Move_Flag_Left = 0, right_Move_Flag_Left = 0, delay = 0;

	int landing_Flag_Right = 0, left_Flag_Right = 0, right_Flag_Right = 0, forward_Flag_Right = 0, backward_Flag_Right = 0, left_Move_Flag_Right = 0, right_Move_Flag_Right = 0;

	/////////////////////////	Connection	/////////////////////////////////////
	///*
	WSADATA		 WSAData;
	SOCKADDR_IN addr;
	SOCKET s;
	char buffer[MAXLINE];
	char angle[MAXLINE], x[MAXLINE], y[MAXLINE];
	int readbytes;
	int i, len;
	float x_point = 1.0, y_point = 2.0, angle_size = 3.0;

	u_long on = TRUE;

	if (WSAStartup(MAKEWORD(2, 0), &WSAData) != 0)
	{
		return 1;
	}
	//*/
	///////////////////////////////////////////////////////////////////////////////

	while (waitKey(10 != 'm')) {	



		int flag = 0;

		Point center_Point = Point(0, 0);

		kinect.Run();

		int finger_Left[5] = { 0, };

		int finger_Right[5] = { 0, };

		if (kinect.finger_Result_Left.size() != 0)
		{
			for (int i = 0; i < kinect.finger_Result_Left.size(); i++)
			{
				//cout << kinect.finger_Result_Left[i].x << "	" << kinect.finger_Result_Left[i].y << endl;
				if (kinect.finger_Result_Left[i].y == 0)
				{
					finger_Left[kinect.finger_Result_Left[i].x - 1] = 1;
				}
				else if (kinect.finger_Result_Left[i].y != 0)
				{
					for (int j = kinect.finger_Result_Left[i].x; j <= kinect.finger_Result_Left[i].y; j++)
					{
						finger_Left[j - 1] = 1;
					}
				}
				
			}

			//cout << finger[0] << "	" << finger[1] << "	" << finger[2] << "	" << finger[3] << "	" << finger[4] << "	" << endl;

		}
		if (kinect.finger_Result_Right.size() != 0)
		{
			for (int i = 0; i < kinect.finger_Result_Right.size(); i++)
			{
				//cout << kinect.finger_Result_Left[i].x << "	" << kinect.finger_Result_Left[i].y << endl;
				if (kinect.finger_Result_Right[i].y == 0)
				{
					finger_Right[kinect.finger_Result_Right[i].x - 1] = 1;
				}
				else if (kinect.finger_Result_Right[i].y != 0)
				{
					for (int j = kinect.finger_Result_Right[i].x; j <= kinect.finger_Result_Right[i].y; j++)
					{
						finger_Right[j - 1] = 1;
					}
				}

			}

			//cout << finger[0] << "	" << finger[1] << "	" << finger[2] << "	" << finger[3] << "	" << finger[4] << "	" << endl;

		}

		if ((finger_Right[0] == 1) && (finger_Right[1] == 0) && (finger_Right[2] == 0) && (finger_Right[3] == 0) && (finger_Right[4] == 1))
		{
			temp_Time = GetTickCount();

			if (start == 0)
				start = temp_Time;
			else if ((start != 0) && ((temp_Time - start) / 1000) > 1)
			{
				start = 0;

				landing_Flag_Right = 1;

				cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

				flag = 1;
			}
		}
		else if ((finger_Right[0] == 0) && (finger_Right[1] == 1) && (finger_Right[2] == 1) && (finger_Right[3] == 0) && (finger_Right[4] == 0))
		{
			left_Flag_Right = 1;

			cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

			flag = 2;
		}
		else if ((finger_Right[0] == 0) && (finger_Right[1] == 1) && (finger_Right[2] == 1) && (finger_Right[3] == 1) && (finger_Right[4] == 0))
		{
			right_Flag_Right = 1;

			cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

			flag = 3;
		}
		else if ((finger_Right[0] == 0) && (finger_Right[1] == 1) && (finger_Right[2] == 0) && (finger_Right[3] == 0) && (finger_Right[4] == 1))
		{
			left_Move_Flag_Right = 1;

			cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

			flag = 4;
		}
		else if ((finger_Right[0] == 1) && (finger_Right[1] == 1) && (finger_Right[2] == 0) && (finger_Right[3] == 0) && (finger_Right[4] == 1))
		{
			right_Move_Flag_Right = 1;

			cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

			flag = 5;
		}
		else if ((finger_Right[0] == 0) && (finger_Right[1] == 1) && (finger_Right[2] == 0) && (finger_Right[3] == 0) && (finger_Right[4] == 0))
		{
			forward_Flag_Right = 1;

			cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

			flag = 6;
		}
		else if ((finger_Right[0] == 0) && (finger_Right[1] == 1) && (finger_Right[2] == 1) && (finger_Right[3] == 1) && (finger_Right[4] == 1))
		{
			backward_Flag_Right = 1;

			cout << finger_Right[0] << "	" << finger_Right[1] << "	" << finger_Right[2] << "	" << finger_Right[3] << "	" << finger_Right[4] << "	" << endl;

			flag = 7;
		}




		if ((finger_Left[0] == 1) && (finger_Left[1] == 0) && (finger_Left[2] == 0) && (finger_Left[3] == 0) && (finger_Left[4] == 1))
		{
			temp_Time = GetTickCount();

			if (start == 0)
				start = temp_Time;
			else if ((start != 0) && ((temp_Time - start) / 1000) > 1)
			{
				start = 0;

				landing_Flag_Left = 1;

				cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

				//flag = 1;
			}
		}
		else if ((finger_Left[0] == 0) && (finger_Left[1] == 0) && (finger_Left[2] == 1) && (finger_Left[3] == 1) && (finger_Left[4] == 0))
		{
			left_Flag_Left = 1;

			cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

			//flag = 2;
		}
		else if ((finger_Left[0] == 0) && (finger_Left[1] == 1) && (finger_Left[2] == 1) && (finger_Left[3] == 1) && (finger_Left[4] == 0))
		{
			right_Flag_Left = 1;

			cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

			//flag = 3;
		}
		else if ((finger_Left[0] == 1) && (finger_Left[1] == 0) && (finger_Left[2] == 0) && (finger_Left[3] == 1) && (finger_Left[4] == 0))
		{
			left_Move_Flag_Left = 1;

			cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

			//flag = 4;
		}
		else if ((finger_Left[0] == 1) && (finger_Left[1] == 0) && (finger_Left[2] == 0) && (finger_Left[3] == 1) && (finger_Left[4] == 1))
		{
			right_Move_Flag_Left = 1;

			cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

			//flag = 5;
		}
		else if ((finger_Left[0] == 0) && (finger_Left[1] == 0) && (finger_Left[2] == 0) && (finger_Left[3] == 1) && (finger_Left[4] == 0))
		{
			forward_Flag_Left = 1;

			cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

			//flag = 6;
		}
		else if ((finger_Left[0] == 1) && (finger_Left[1] == 1) && (finger_Left[2] == 1) && (finger_Left[3] == 1) && (finger_Left[4] == 0))
		{
			backward_Flag_Left = 1;

			cout << finger_Left[0] << "	" << finger_Left[1] << "	" << finger_Left[2] << "	" << finger_Left[3] << "	" << finger_Left[4] << "	" << endl;

			//flag = 7;
		}


		///////////////////////////	For Connection	/////////////////////////////////////////////////
		///*
		s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

		if (s == INVALID_SOCKET)
		{
			return 1;
		}

		//ioctlsocket(s, FIONBIO, &on);

		//int optval = 1;
		//setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*)&optval, (int)sizeof(optval));

		addr.sin_family = AF_INET;
		addr.sin_port = htons(PORT);
		addr.sin_addr.S_un.S_addr = inet_addr(IP);

		if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
		{
			printf("fail to connect\n");
		}

		sprintf(buffer, "%d", flag);

		len = strlen(buffer);

		send(s, buffer, len + 1, 0);

		printf("send messages (%d bytes)\n", len);
		send(s, buffer, len, 0);

		for (readbytes = 0; readbytes<len;)
			readbytes += recv(s, buffer + readbytes, len - readbytes, 0);

		printf("recv messages = %s\n", buffer);

		//*/
		///////////////////////////////////////////////////////////////////////////////////////////////



        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();
	

        // Take off / Landing 
		if ((key == ' ') || (landing_Flag_Left == 1)){
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
			landing_Flag_Left = 0;

			flag = 0;
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if ((key == 'i' || key == CV_VK_UP) || (forward_Flag_Left == 1))
		{
			vx = 0.5;
		}
		if ((key == 'k' || key == CV_VK_DOWN) || (backward_Flag_Left == 1))
		{
			vx = -0.5;
		}
		if ((key == 'u' || key == CV_VK_LEFT) || (left_Flag_Left == 1))
		{
			vr = 0.5;
		}
		if ((key == 'o' || key == CV_VK_RIGHT) || (right_Flag_Left == 1))
		{
			vr = -0.5;
		}
		if ((key == 'j') || (left_Move_Flag_Left == 1))
		{
			vy = 0.5;
		}
		if ((key == 'l') || (right_Move_Flag_Left == 1))
		{
			vy = -0.5;
		}
		if (key == 'q')
		{
			vz = 1.0;
		}
		if (key == 'a')
		{
			vz = -1.0;
		}

		ardrone.move3D(vx, vy, vz, vr);


		//	Reset flags
		{
			left_Flag_Left = 0;

			right_Flag_Left = 0;

			left_Move_Flag_Left = 0;

			right_Move_Flag_Left = 0;

			forward_Flag_Left = 0;

			backward_Flag_Left = 0;

			left_Flag_Right = 0;

			right_Flag_Right = 0;

			left_Move_Flag_Right = 0;

			right_Move_Flag_Right = 0;

			forward_Flag_Right = 0;

			backward_Flag_Right = 0;

		}

		

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);

    }

    // See you
    ardrone.close();

    return 0;
}