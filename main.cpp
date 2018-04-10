#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "VideoFaceDetector.h"
/* server.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

/* Headerfiles für UNIX/Linux */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

//#include <curses.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* Portnummer */
#define PORT 1233


const cv::String    WINDOW_NAME("Camera video");
const cv::String    CASCADE_FILE("/home/ferber/maker_ws/src/face_detect_n_track/haarcascade_frontalface_default.xml");


static void echo( int );

static void error_exit(char *errorMessage);

/* Die Funktion gibt Daten vom Client auf stdout aus,
 * die dieser mit der Kommandozeile übergibt. */
static bool transmit(int client_socket, cv::Point point)
{
    char echo_string[30];
    /* Länge der Eingabe */
    sprintf(echo_string, "%i %i\n", point.x, point.y);
    int echo_len = strlen(echo_string);
    printf("face found %s %i \n",echo_string,echo_len);
    fflush(stdout);
    /* den String inkl. Nullterminator an den Server senden */
    if (send(client_socket, echo_string, echo_len, 0) != echo_len)
		return false;
        //error_exit("send() hat eine andere Anzahl"
        //           " von Bytes versendet als erwartet !!!!");
	return true;
    
}

/* Die Funktion gibt den aufgetretenen Fehler aus und
 * beendet die Anwendung. */
static void error_exit(char *error_message) {

    fprintf(stderr, "%s: %s\n", error_message, strerror(errno));

    exit(EXIT_FAILURE);
}

static int server_init() {
    struct sockaddr_in server, client;

    int sock;
    /* Erzeuge das Socket. */
    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0)
        error_exit("Fehler beim Anlegen eines Sockets");

    /* Erzeuge die Socketadresse des Servers. */
    memset( &server, 0, sizeof (server));
    /* IPv4-Verbindung */
    server.sin_family = AF_INET;
    /* INADDR_ANY: jede IP-Adresse annehmen */
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    /* Portnummer */
    server.sin_port = htons(PORT);

    /* Erzeuge die Bindung an die Serveradresse
     * (genauer: an einen bestimmten Port). */
    if(bind(sock,(struct sockaddr*)&server, sizeof( server)) < 0)
        error_exit("Kann das Socket nicht \"binden\"");

    /* Teile dem Socket mit, dass Verbindungswünsche
     * von Clients entgegengenommen werden. */
    if(listen(sock, 5) == -1 )
         error_exit("Fehler bei listen");

    printf("Server bereit - wartet auf Anfragen ...\n");
    /* Bearbeite die Verbindungswünsche von Clients
     * in einer Endlosschleife.
     * Der Aufruf von accept() blockiert so lange,
     * bis ein Client Verbindung aufnimmt. */
    //for (;;) {
    
    
    return sock;
}

static int client_accept(int sock){
    struct sockaddr_in client;
    
    unsigned int len = sizeof(client);
    int fd = accept(sock, (struct sockaddr*)&client, &len);
    if (fd < 0)
        error_exit("Fehler bei accept");
    printf("Bearbeite den Client mit der Adresse: %s\n",
       inet_ntoa(client.sin_addr));
    return fd;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
  	ros::NodeHandle nh;
  	image_transport::ImageTransport it(nh);
  	image_transport::Publisher pub = it.advertise("camera/image_eyes", 1);
	// Try opening camera
	cv::VideoCapture camera(0);
	//cv::VideoCapture camera("D:\\video.mp4");
	if (!camera.isOpened()) {
		fprintf(stderr, "Error getting camera...\n");
		exit(1);
	}

	//cv::namedWindow(WINDOW_NAME, cv::WINDOW_KEEPRATIO | cv::WINDOW_AUTOSIZE);

	VideoFaceDetector detector(CASCADE_FILE, camera);
	cv::Mat frame;
	double fps = 0, time_per_frame;
    // if all is set up --> open server socket to transmit face position
    int server_sock = server_init();
    // wait for client to connect
    bool running =true;
    while (running){
	    int client_sock = client_accept(server_sock);
		while (true)
		{
			auto start = cv::getCPUTickCount();
			detector >> frame;
			auto end = cv::getCPUTickCount();
	
			time_per_frame = (end - start) / cv::getTickFrequency();
			fps = (15 * fps + (1 / time_per_frame)) / 16;

			printf("Time per frame: %3.3f\tFPS: %3.3f\n", time_per_frame, fps);

			if (detector.isFaceFound())
			{
				cv::rectangle(frame, detector.face(), cv::Scalar(255, 0, 0));
                		cv::Point point = detector.facePosition();
				cv::circle(frame, point, 30, cv::Scalar(0, 255, 0));
				//cv::imshow(WINDOW_NAME, frame);
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
				pub.publish(msg);
                // transmit point to client
                if(!transmit(client_sock, point)){
					break;
				}
			}
			
			
			if(cv::waitKey(25) == 27){
				running =false;
				break;
			}
		}
        if(close(client_sock) == -1)
            error_exit("Fehler bei close Client");
	}
    if(close(server_sock) == -1)
            error_exit("Fehler bei close Server");
	return 0;
}
