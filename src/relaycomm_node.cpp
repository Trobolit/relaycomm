#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"

#include <string>

#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

#define BAUDRATE B9600
#define SUB_CHANEL "relay_controll"
int fileDescriptor;

unsigned char serialData[] = {250, 1, 1, 1, 0, 251};

void relayCallback(const std_msgs::Int8MultiArray::ConstPtr& array)
{
		serialData[1] = array->data[0];
		serialData[2] = array->data[1];
		serialData[3] = array->data[2];
		serialData[4] = array->data[3];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relaycomm");
  ros::NodeHandle n;
  ros::Rate loop_rate(2);

  ros::Subscriber relay = n.subscribe<std_msgs::Int8MultiArray>(SUB_CHANEL, 10, relayCallback);	
  // The serial setup is mostly inspired by this:
  // https://stackoverflow.com/questions/38533480/c-libserial-serial-connection-to-arduino
  // Which sets things like parity, stop byte, length, etc.

  // This sets up the serial communication to the arduino driver.
    fileDescriptor = open("/dev/ttyACM0", O_RDWR | O_NOCTTY); //open link to arudino


    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    // set to 8N1
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;

    newtio.c_iflag = IGNPAR;

    // output mode to
    //newtio.c_oflag = 0;
    newtio.c_oflag |= OPOST;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 10; /* inter-character timer 1 sec */
    newtio.c_cc[VMIN] = 0; /* blocking read disabled  */

    tcflush(fileDescriptor, TCIFLUSH);
    if (tcsetattr(fileDescriptor, TCSANOW, &newtio)) {
        perror("could not set the serial settings!");
        return -99;
    }

    long x = 0;

    while(ros::ok()){
	
	//ROS_INFO("Setting relays...");

	x = write(fileDescriptor, serialData , 6*sizeof(char));

	//ROS_INFO("Setting relays, sent %d bytes.", x);
	//ROS_INFO("%d %d %d %d %d %d", serialData[0],serialData[1],serialData[2],serialData[3],serialData[4],serialData[5]);

    	ros::spinOnce();
    	loop_rate.sleep();
    }


  return 0;
}

