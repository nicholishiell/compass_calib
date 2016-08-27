// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Standard C headers
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <fstream>

using namespace std;


const int HMC5883L_I2C_ADDR = 0x1E;

#define CONFIG_A 0x00
#define CONFIG_B 0x01
#define MODE 0x02
#define DATA 0x03 //read 6 bytes: x msb, x lsb, z msb, z lsb, y msb, y lsb

void selectDevice(int fd, int addr, char * name){
  
    if (ioctl(fd, I2C_SLAVE, addr) < 0){
      
        fprintf(stderr, "%s not present\n", name);
        //exit(1);
    }
}

void writeToDevice(int fd, int reg, int val){
  
    char buf[2];
    buf[0]=reg;
    buf[1]=val;

    if (write(fd, buf, 2) != 2)
    {
        fprintf(stderr, "Can't write to HMC5883L\n");
        //exit(1);
    }
}

int main(int argc, char **argv){
  // Setup ROS node and publisher
  ros::init(argc, argv, "compass_node");

  ros::NodeHandle n;
 
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(25);

  // Setup communication with compass
  int fd;
  unsigned char buf[16];
  float heading;
 
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0){
        // Open port for reading and writing
        fprintf(stderr, "Failed to open i2c bus\n");

        return 1;
   }

  selectDevice(fd, HMC5883L_I2C_ADDR, "HMC5883L");

  // Set to average 8 samples.
  writeToDevice(fd, CONFIG_A,  0b01110100);
  // Set range to max/min +/-1.3 Gauss.
  writeToDevice(fd, CONFIG_B, 0b11100000);
  // Set sensor to continous measurement mode.
  writeToDevice(fd, MODE, 0b00000000);

  geometry_msgs::Twist msg;

  msg.linear.x = 0.;
  msg.angular.z = 1.;
  msg.angular.y = -1;

  short xLow = 32767;
  short xHigh = -32768;
  short yLow = 32767;
  short yHigh = -32768;

  int counter = 0;
  
  while (ros::ok() and counter < 300){
    buf[0] = 0x03;
    cmd_vel_pub.publish(msg);
    
    if ((write(fd, buf, 1)) != 1){
      // Send the register to read from
      fprintf(stderr, "Error writing to i2c slave\n");
    }

    if (read(fd, buf, 6) != 6) {
      fprintf(stderr, "Unable to read from HMC5883L\n");
    }
    else {
      short x = (buf[0] << 8) | buf[1];
      short y = (buf[4] << 8) | buf[5];
      short z = (buf[2] << 8) | buf[3];

      if(x < xLow) xLow = x;
      if(x > xHigh) xHigh = x;

      if(y < yLow) yLow = y;
      if(y > yHigh) yHigh = y;

      //printf("%d: [%d,%d]\t[%d,%d]\n", counter, xLow, xHigh, yLow, yHigh);     
    }
    
    counter++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  msg.linear.x = 0.;
  msg.angular.z = 0.;
  msg.angular.y = -1;
  counter = 0;
  while (ros::ok() and counter < 10){
    counter++;
    cmd_vel_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  printf("[xLow, xHigh] = %d\t%d\n", xLow, xHigh);
  printf("[yLow, yHigh] = %d\t%d\n", yLow, yHigh);

  fstream output;
  output.open("/home/pi/ns_catkin_ws/calibData/compass_calib", std::fstream::out);
  output << xLow <<" "<< xHigh<<"\n";
  output << yLow <<" "<< yHigh<<"\n";
  output.close();
  printf("Done!\n");
  return 0;
}
