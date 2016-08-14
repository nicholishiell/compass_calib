// ROS headers
#include "ros/ros.h"
#include "std_msgs/Float64.h"

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
#include <math.h>

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
 
  ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("currentHeading", 1000);

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

  while (ros::ok()){
    buf[0] = 0x03;
    std_msgs::Float64 msg;

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

      int xMin = -110;
      int xMax = 89;

      int yMin =-150;
      int yMax = 64;
      
      float xScaled = 2.*(float)(x - xMin) / (xMax - xMin) - 1.0;
      float yScaled = 2.*(float)(y - yMin) / (yMax - yMin) - 1.0;
      
      //      heading = atan2(y, x) * 180. / M_PI;
      heading = atan2(yScaled, xScaled) * 180. / M_PI;
      
      msg.data = heading;
    
      heading_pub.publish(msg);
      //printf("%d\t%d\t%f\n", x,y,heading);
    }
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}
