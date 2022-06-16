#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <float.h>
#include <math.h>

using namespace std;

#define LEFT           	  0      // Swing direction
#define _L		  LEFT
#define L		  LEFT
#define RIGHT             1
#define _R		  RIGHT
#define R		  RIGHT
#define _X		  0
#define _Y		  1
#define _THETA            2
#define LW		  0
#define RW		  1

#define ENABLE            1
#define DISABLE           0
#define FAIL              0
#define SUCCESS           1

#define ON                1
#define OFF               0
#define RESET             0

#define CCW               -1
#define CW                1

#define Abs(a)            (((a)<(0)) ? -(a):(a))

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned int   DWORD;

//v1.9b add roslaunch parameter 'RMID'
//      modify the code for sending PID 224 in main.cpp
