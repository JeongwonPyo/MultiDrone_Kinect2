#define WIN32_LEAN_AND_MEAN

#include <WinSock2.h>
#include <WS2tcpip.h>
#include <stdio.h>
#include <stdlib.h>


// link with Ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

#define DEFAULT_PORT "27015"
#define DEFAULT_BUFFER_LENGTH	512

#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int RecvData(SOCKET socket, LPSTR IpszBuf, int nBufSize)
{
	if (NULL == socket || NULL == IpszBuf)
	{
		return SOCKET_ERROR;
	}
	int nByteRecv = 0;
	while (1)
	{
		nByteRecv = recv(socket, IpszBuf, nBufSize, 0);
		if (SOCKET_ERROR == nByteRecv)
		{
			if (WSAEWOULDBLOCK == WSAGetLastError())
			{
				continue;
			}
			else
			{
				break;
			}
		}
		else
		{
			break;
		}
	}
	return nByteRecv;
}

int main(int argc, char *argv[])
{
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

	//////////////////	Connection	////////////////////////////////

	u_long on = TRUE;

	WSADATA wsaData;

	struct sockaddr_in server_addr, client_addr;

	int nAcceptClientInfo;

	int sp;

	// Initialize Winsock
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
	{
		printf("WSAStartup failed: %d\n", iResult);
		return 1;
	}

	struct addrinfo	*result = NULL,
		hints;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;		// Internet address family is unspecified so that either an IPv6 or IPv4 address can be returned
	hints.ai_socktype = SOCK_STREAM;	// Requests the socket type to be a stream socket for the TCP protocol
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the local address and port to be used by the server
	iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	if (iResult != 0)
	{
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	SOCKET ListenSocket = INVALID_SOCKET;

	// Create a SOCKET for the server to listen for client connections
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);

	if (ListenSocket == INVALID_SOCKET)
	{
		printf("Error at socket(): %d\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return 1;
	}

	ioctlsocket(ListenSocket, FIONBIO, &on);

	int optival = 1;
	setsockopt(ListenSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&optival, (int)sizeof(optival));

	// Setup the TCP listening socket
	iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);

	if (iResult == SOCKET_ERROR)
	{
		printf("bind failed: %d", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	freeaddrinfo(result);

	// To listen on a socket
	if (listen(ListenSocket, SOMAXCONN) == SOCKET_ERROR)
	{
		printf("listen failed: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	SOCKET ClientSocket;

	ClientSocket = INVALID_SOCKET;


	///////////////////////////////////////////////////////////////

	int flag = 0;

    while (1) {

		//////////////	Connect Section	//////////////////////////

		ZeroMemory(&client_addr, sizeof(struct sockaddr_in));

		nAcceptClientInfo = sizeof(struct sockaddr_in);

		// Accept a client socket
		ClientSocket = accept(ListenSocket, (struct sockaddr*) &client_addr, &nAcceptClientInfo);

		char recvbuf[DEFAULT_BUFFER_LENGTH];
		int iSendResult;

		iResult = RecvData(ClientSocket, recvbuf, DEFAULT_BUFFER_LENGTH);

		if (iResult > 0)
		{
			char msg[DEFAULT_BUFFER_LENGTH];
			memset(&msg, 0, sizeof(msg));
			strncpy(msg, recvbuf, iResult);

			printf("Received: %s\n", msg);

			flag = atoi(msg);

			iSendResult = send(ClientSocket, recvbuf, iResult, 0);

			printf("Bytes sent: %ld\n", iSendResult);
		}
		else if (iResult == 0)
			printf("Connection closed\n");

		////////////////////////////////////////////////////////////

        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Take off / Landing 
        if ((key == ' ') || (flag == 1)) {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if ((key == 'i' || key == CV_VK_UP) || (flag == 6))    vx = 1.0;
		if ((key == 'k' || key == CV_VK_DOWN) || (flag == 7))  vx = -1.0;
		if ((key == 'u' || key == CV_VK_LEFT) || (flag == 2))
		{
			vr = 1.0;
		}
		if ((key == 'o' || key == CV_VK_RIGHT) || (flag == 3))
		{
			vr = -1.0;
		}
		if ((key == 'j') || (flag == 4)) vy = 1.0;
		if ((key == 'l') || (flag == 5)) vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
		ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);
    }

	// Free the resouces
	closesocket(ListenSocket);
	WSACleanup();

    // See you
    ardrone.close();

    return 0;
}