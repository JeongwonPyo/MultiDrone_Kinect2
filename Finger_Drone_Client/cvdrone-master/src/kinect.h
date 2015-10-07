
#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#endif

// Windows Header Files
#include <windows.h>

#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>

// Kinect Header files
#include <Kinect.h>

#pragma comment (lib, "d2d1.lib")

#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//#include <opencv\cv.h>
//#include <opencv\highgui.h>
//#include <opencv2\core\core.hpp>
//#include <opencv2\opencv.hpp>
//#include <opencv2\imgproc\imgproc.hpp>
//#include <opencv2\imgproc\imgproc_c.h>
//#include <opencv2\features2d\features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <math.h>

using namespace std;

using namespace cv;

struct FEMD{
	Point start;
	Point end;
	int Size;
};

class CBodyBasics
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CBodyBasics();

	/// <summary>
	/// Destructor
	/// </summary>
	~CBodyBasics();

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	//static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);


	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	//LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	int                     Run();

	//////////////////////////	OpenCV Parameter	///////////////////////////

	int hand_Size;

	int hand_Size_Color;

	int hand_Depth_Range;

	Mat OpenCV_View;

	Mat depth_Data = Mat::zeros(cDepthHeight, cDepthWidth, CV_32F);

	Mat depth_Data_Color = Mat::zeros(cColorHeight, cColorWidth, CV_32F);

	Mat depth_View = Mat::zeros(cDepthHeight, cDepthWidth, CV_8U);

	Mat depth_View_Color = Mat::zeros(cColorHeight, cColorWidth, CV_8U);

	Mat color_View = Mat::zeros(cColorHeight, cColorWidth, CV_8UC4);

	Mat RGB_Depth_Map = Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC4);

	Mat left_Hand;

	Mat right_Hand;

	Mat left_Hand_Color;

	Mat right_Hand_Color;

	Mat left_Hand_Color_In_Depth;

	Mat right_Hand_Color_In_Depth;

	Mat show_Right_Hand;

	Mat show_Left_Hand;

	Point R_Hand;

	Point L_Hand;

	Point left_For_Edge;

	Point right_For_Edge;

	Point R_Hand_Color;

	Point L_Hand_Color;

	vector<FEMD> base_Left, base_Right, recent_Left, recent_Right;

	vector<Point> finger_Result_Left, finger_Result_Right;

	D2D1_POINT_2F jointPoints_For_Hand[JointType_Count];
	//////////////////////////////////////////////////////////////////////////////

private:
	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	INT64                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;
	DepthSpacePoint*        m_pDepthCoordinates;

	// Body reader
	IBodyFrameReader*       m_pBodyFrameReader;

	// Color reader
	IColorFrameReader*      m_pColorFrameReader;

	// Body/hand drawing
	ID2D1HwndRenderTarget*  m_pRenderTarget;
	ID2D1SolidColorBrush*   m_pBrushJointTracked;
	ID2D1SolidColorBrush*   m_pBrushJointInferred;
	ID2D1SolidColorBrush*   m_pBrushBoneTracked;
	ID2D1SolidColorBrush*   m_pBrushBoneInferred;
	ID2D1SolidColorBrush*   m_pBrushHandClosed;
	ID2D1SolidColorBrush*   m_pBrushHandOpen;
	ID2D1SolidColorBrush*   m_pBrushHandLasso;


	// Depth reader
	IDepthFrameReader*      m_pDepthFrameReader;

	// Direct2D
	//ImageRenderer*          m_pDrawDepth;
	//ImageRenderer*          m_pDrawColor;
	ID2D1Factory*           m_pD2DFactory;
	//RGBQUAD*                m_pDepthRGBX;
	RGBQUAD*                m_pColorRGBX;
	RGBQUAD*                m_pBackgroundRGBX;
	RGBQUAD*                m_pOutputRGBX;

	/// <summary>
	/// Main processing function
	/// </summary>
	void                    Update();

	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 InitializeDefaultSensor();

	/// <summary>
	/// Handle new body data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="nBodyCount">body data count</param>
	/// <param name="ppBodies">body data in frame</param>
	/// </summary>
	void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

	/// <summary>
	/// Set the status bar message
	/// </summary>
	/// <param name="szMessage">message to display</param>
	/// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
	/// <param name="bForce">force status update</param>
	//bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	/// <summary>
	/// Ensure necessary Direct2d resources are created
	/// </summary>
	/// <returns>S_OK if successful, otherwise an error code</returns>
	//HRESULT EnsureDirect2DResources();

	/// <summary>
	/// Dispose Direct2d resources 
	/// </summary>
	void DiscardDirect2DResources();

	/// <summary>
	/// Converts a body point to screen space
	/// </summary>
	/// <param name="bodyPoint">body point to tranform</param>
	/// <param name="width">width (in pixels) of output buffer</param>
	/// <param name="height">height (in pixels) of output buffer</param>
	/// <returns>point in screen-space</returns>
	D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);

	/// <summary>
	/// Draws a body 
	/// </summary>
	/// <param name="pJoints">joint data</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	void                    DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);

	/// <summary>
	/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
	/// </summary>
	/// <param name="handState">state of the hand</param>
	/// <param name="handPosition">position of the hand</param>
	void                    DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

	/// <summary>
	/// Draws one bone of a body (joint to joint)
	/// </summary>
	/// <param name="pJoints">joint data</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	/// <param name="joint0">one joint of the bone to draw</param>
	/// <param name="joint1">other joint of the bone to draw</param>
	void                    DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);

	/// <summary>
	/// Handle new depth data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// <param name="nMinDepth">minimum reliable depth</param>
	/// <param name="nMaxDepth">maximum reliable depth</param>
	/// </summary>
	void                    ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

	/// <summary>
	/// Handle new color data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// </summary>
	void                    ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);


	/// <summary>
	/// Load an image from a resource into a buffer
	/// </summary>
	/// <param name="resourceName">name of image resource to load</param>
	/// <param name="resourceType">type of resource to load</param>
	/// <param name="nOutputWidth">width (in pixels) of scaled output bitmap</param>
	/// <param name="nOutputHeight">height (in pixels) of scaled output bitmap</param>
	/// <param name="pOutputBuffer">buffer that will hold the loaded image</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 LoadResourceImage(PCWSTR resourceName, PCWSTR resourceType, UINT nOutputWidth, UINT nOutputHeight, RGBQUAD* pOutputBuffer);

	void					RGB_Depth_Mapping(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, Mat Color_View, Mat depth_Data);

	void					Cut_Hand_Color_Float(Mat depth_Data, Mat left_Hand, Mat right_Hand, Point L_Hand, Point R_Hand, D2D1_POINT_2F jointPoints_For_Hand[], int size, float range);

};
