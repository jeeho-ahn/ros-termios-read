/*
  This is an example code for reading a simple serial communication through a serial port.
  Tested with a ROBOTIS OpenCM, and will likely to work with Arduino and any other devices that talks through serial ports as well.
  This version is intended to read a single signal comming from a interrupted tact switch on a MCU.
  Setting parameters through a ros launch file is recommended.

  This ROS Package is piled up on the termios code usaged explained here: http://xanthium.in/Serial-Port-Programming-on-Linux

  created by Ahn, Jeeho
  2018.04.07
*/


#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <iostream>
#include <string>

#include <std_msgs/Bool.h>


ros::Publisher* pub_SinPointer;
bool will_continue = false;

std::string s_port = "/dev/ttyACM0";
std::string b_rate = "9600";
bool input_monitor = true;  //

int main(int argc, char** argv)
{
    std::string node_name = "serial_read_ex";

    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle nh;

    ros::Publisher pub_Sin = nh.advertise<std_msgs::Bool>
        ("/pub_Sin", 2);


    pub_SinPointer = &pub_Sin;
    //////////////////////////////////////////////////////////////

    nh.param(node_name + "/port", s_port, std::string("/dev/ttyACM0"));
    nh.param(node_name + "/baud", b_rate, std::string("9600"));
    nh.param(node_name + "/input_monitor", input_monitor, true);

    int fd;
      fd = open(s_port.c_str(),O_RDWR | O_NOCTTY);
      if(fd == -1)
         std::cout << std::endl << "Error Opening " << s_port << std::endl;
      else
         {std::cout << std::endl << "Successfully opened " << s_port << std::endl; will_continue=true;}

      if(will_continue==true)
      {
        struct termios SerialPortSettings;
        //tcgetattr(fd, &SerialPortSettings);

        cfsetispeed(&SerialPortSettings,B9600);  //set reading speed
        cfsetospeed(&SerialPortSettings,B9600);  //set writing speed

        SerialPortSettings.c_cflag &= ~PARENB;  //no parity
        SerialPortSettings.c_cflag &= ~CSTOPB;  //Stop bit 1
        SerialPortSettings.c_cflag &= ~CSIZE;
        SerialPortSettings.c_cflag |= CS8;

        SerialPortSettings.c_cflag &= ~CRTSCTS; //HW flow control off
        SerialPortSettings.c_cflag |= CREAD | CLOCAL; //turn on receiver
        SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
        SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        tcsetattr(fd,TCSANOW,&SerialPortSettings); //set

        tcgetattr(fd, &SerialPortSettings);
        SerialPortSettings.c_cc[VMIN] = 0; //number of char to read
        SerialPortSettings.c_cc[VTIME]=10;  //timeout 1 sec
        tcsetattr(fd,TCSANOW,&SerialPortSettings); //set

        char read_buffer[32];
        int bytes_read = 0;

        memset(read_buffer, 0, sizeof(read_buffer));

        std_msgs::Bool state; state.data=true;

    while(ros::ok())
    {
      bytes_read = read(fd,&read_buffer,sizeof(read_buffer));

      if(bytes_read!=0)
      {
        //state.data = true;
        pub_SinPointer->publish(state);
        if(input_monitor==true)
          std::cout << read_buffer << std::endl;
        //state.data = false;
      }

      // ros::spinOnce();
      //loop_rate.sleep();
      memset(read_buffer, 0, sizeof(read_buffer));
    }
   }
    std::cout << std::endl;
    close(fd);
    return 0;
}
