#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <string>

#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

#define BAUDRATE B9600

int fileDescriptor;

unsigned char serialData[] = {250, 1, 1, 1, 0, 251};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relaycomm");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  // The serial setup is mostly inspired by this:
  // https://stackoverflow.com/questions/38533480/c-libserial-serial-connection-to-arduino
  // Which sets things like parity, stop byte, length, etc.

  // This sets up the serial communication to the arduino driver.
    fileDescriptor = open("/dev/ttyACM1", O_RDWR | O_NOCTTY); //open link to arudino


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

    while(ros::ok()){

	write(fileDescriptor, serialData , 6*sizeof(char));

    	ros::spinOnce();
    	loop_rate.sleep();
    }


  return 0;
}

