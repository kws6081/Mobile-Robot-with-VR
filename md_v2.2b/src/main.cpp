///v1.9d modifying code while PNT_TQ_OFF(PNT_BREKE) communication. it request PNT_MAIN_DATA and for 2s delay
///v1.9e adding code reset command after 2s
///v1.9f adding code reset command when RMID is MDT'
///v1.9g modifying 'Md.sCmdAngularVel calculation' in cmd_main.cpp according to "md_node/angleresolution"
#include "md/global.hpp"
#include "md/main.hpp"
#include "md/com.hpp"
#include "md/robot.hpp"

#include "md/vel_msg.h"
#include "md/monitor_msg.h"

#include <ros/ros.h>

TableOfRobotControlRegister RB;
Communication Com;
LOG Log;
//
//It is a message callback function.
//It is a function that oprates when a topic message named 'vel_topic' is received.
//The input message is to receive the vel_msg message from 'md' package in msg directory
void velCallBack(const md::vel_msg::ConstPtr& vel)
{
    Com.nCmdSpeed       = -1*(vel->nLinear);
    Com.nCmdAngSpeed    = -1*(vel->nAngular);
    Com.fgResetOdometry = vel->byResetOdometry;
    Com.fgResetAngle    = vel->byResetAngle;
    Com.fgResetAlarm    = vel->byResetAlarm;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "md_node");                                                   //Node name initialization.
    ros::NodeHandle nh;                                                                 //Node handle declaration for communication with ROS system.
    ros::Subscriber vel_sub = nh.subscribe("vel_topic", 100, velCallBack);              //Subscriber declaration.
    ros::Publisher monitor_pub = nh.advertise<md::monitor_msg>("monitor_topic", 100);   //Publisher declaration.
    ros::Publisher string_pub = nh.advertise<std_msgs::String>("string_com_topic", 100);

    md::monitor_msg monitor;                                                            //monitor_msg declares message 'message' as message file.
    ros::Rate r(10000);                                                                 //Set the loop period -> 100us.

    //variable declaration
    IByte iData;
    static BYTE byCntComStep, byCntCmdVel, fgSendCmdVel, byCntInitStep;
    static BYTE byCnt2500us, byCntCase[10], byFgl, byFglReset, fgInitPosiResetAfter2s, byCntReset;
    static BYTE byCnt, byCntStartDelay, byFglIO;

    int nArray[5];

    Log.fgSet         = ON;
    fgSendCmdVel      = ON;
    byFgl             = OFF;
    Com.fgInitsetting = OFF;

    //Store the value of the parameter in the variable
    nh.getParam("md_node/PC", Com.nIDPC);
    nh.getParam("md_node/MDUI", Com.nIDMDUI);
    nh.getParam("md_node/MDT", Com.nIDMDT);
    nh.getParam("md_node/baudrate", Com.nBaudrate);
    nh.getParam("md_node/diameter", Com.nDiameter);
    nh.getParam("md_node/wheelLength", Com.nWheelLength);
    nh.getParam("md_node/reduction", Com.nGearRatio);
    nh.getParam("md_node/direction", Com.fgDirSign);
    nh.getParam("md_node/halltype", Com.nHallType);
    nh.getParam("md_node/maxrpm", Com.nMaxRPM);
    nh.getParam("md_node/angleresolution", Com.nAngResol);
    nh.getParam("md_node/RMID", Com.nRMID);
    nh.getParam("md_node/slowstart", Com.nSlowstart);
    nh.getParam("md_node/slowdown", Com.nSlowdown);

    InitSerial();     //communication initialization in com.cpp

    while(ros::ok())
    {
        ReceiveDataFromController();

        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;

            if(Com.fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                    case 1:
                        if(++byCntCase[byCntComStep] == TIME_50MS)
                        {
                            byCntCase[byCntComStep] = 0;

                            monitor.lPosiX           = Com.lPosi[_X];
                            monitor.lPosiY           = Com.lPosi[_Y];
                            monitor.sTheta           = Com.sTheta;
                            monitor.sVoltIn          = Com.sVoltIn;
                            monitor.sRealLinearVel   = RB.nRcvLinearVel;
                            monitor.sRealAngularVel  = -1*(RB.nRcvAngularVel);
                            monitor.byUS1            = Com.byUS1;
                            monitor.byUS2            = Com.byUS2;
                            monitor.byUS3            = Com.byUS3;
                            monitor.byUS4            = Com.byUS4;
                            monitor.byPlatStatus     = Com.byPlatStatus;
                            monitor.byDocStatus      = Com.byDocStatus;
                            monitor.byLeftMotStatus  = Com.byStatus[MOT_LEFT];
                            monitor.byRightMotStatus = Com.byStatus[MOT_RIGHT];
                            monitor.sLeftMotCur      = Com.sCurrent[MOT_LEFT];
                            monitor.sRightMotCur     = Com.sCurrent[MOT_RIGHT];
                            monitor.lLeftMotPosi     = Com.lMotorPosi[MOT_LEFT];
                            monitor.lRightMotPosi    = Com.lMotorPosi[MOT_RIGHT];
                            monitor.byLeftIOMonitor  = Com.byIOMonitor[MOT_LEFT];
                            monitor.byRightIOMonitor = Com.byIOMonitor[MOT_RIGHT];

                            monitor_pub.publish(monitor);
                        }
                        break;
                    case 2:
                        if(Com.nRMID == Com.nIDMDUI)
                        {
                            if(++byCntCase[byCntComStep] == TIME_50MS)
                            {
                                byFgl ^= 1;
                                byCntCase[byCntComStep] = 0;

                                if(byFgl == 1)
                                {
                                    nArray[0] = PID_ROBOT_MONITOR2;            //PID 224
                                    PutMdData(PID_REQ_PID_DATA, Com.nIDMDUI, nArray);
                                }
                                else
                                {
                                    if(fgInitPosiResetAfter2s == 0)
                                    {
                                        if(byCntReset++ == 20)
                                        {
                                            fgInitPosiResetAfter2s = 1;
                                            ResetPosture();
                                        }
                                    }

                                    if(Com.fgResetOdometry == 1)
                                    {
                                        if(byFglReset==0) ResetPosture();
                                        byFglReset = 1;
                                    }
                                    else byFglReset = 0;
                                }
                            }
                        }
                        else if(Com.nRMID == Com.nIDMDT)
                        {
                            if(++byCntCase[byCntComStep] == TIME_50MS)
                            {
                                byFgl ^= 1;
                                byCntCase[byCntComStep] = 0;

                                if(byFgl == 0)
                                {
                                    if(fgInitPosiResetAfter2s == 0)
                                    {
                                        if(byCntReset++ == 20)
                                        {
                                            fgInitPosiResetAfter2s = 1;
                                            ResetPosture();
                                        }
                                    }

                                    if(Com.fgResetOdometry == 1)
                                    {
                                        if(byFglReset==0) ResetPosture();
                                        byFglReset = 1;
                                    }
                                    else byFglReset = 0;
                                }
                            }
                        }
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        if(++byCntCase[byCntComStep] == TIME_50MS)
                        {
                            byCntCase[byCntComStep] = 0;

                            byFglIO ^= 1;

                            if(byFglIO)
                            {
                                nArray[0] = PID_PNT_IO_MONITOR;                 //PID 241
                                PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                            }
                            else
                            {
                                if(Com.fgResetAlarm)
                                {
                                    nArray[0] = CMD_ALARM_RESET;
                                    PutMdData(PID_COMMAND, Com.nRMID, nArray);
                                }
                                else
                                {
                                    nArray[0] = PID_PNT_IO_MONITOR;             //PID 241
                                    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                                }
                            }
                        }
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        if(++byCntCase[byCntComStep] == TIME_50MS)
                        {
                            byCntCase[byCntComStep] = 0;
                            if(Com.bRccState == OFF)
                            {
                                if(Com.bEmerSW == ON)
                                {
                                    fgSendCmdVel = OFF;
                                    nArray[0] = ENABLE;
                                    nArray[1] = ENABLE;
                                    nArray[2] = REQUEST_PNT_MAIN_DATA;
                                    PutMdData(PID_PNT_TQ_OFF, Com.nRMID, nArray);
                                }
                                else
                                {
                                    if(fgSendCmdVel)
                                    {
                                        RobotCmd2MotCmd(Com.nCmdSpeed, Com.nCmdAngSpeed);

                                        iData = Short2Byte(RB.sRefRPM[0]);
                                        nArray[0] = iData.byLow;
                                        nArray[1] = iData.byHigh;
                                        iData = Short2Byte(RB.sRefRPM[1]);
                                        nArray[2] = iData.byLow;
                                        nArray[3] = iData.byHigh;
                                        nArray[4] = REQUEST_PNT_MAIN_DATA;
                                        PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray);
                                    }
                                    else
                                    {
                                        if(byCntCmdVel < 40) byCntCmdVel++;
                                        else if(byCntCmdVel == 40) //after 2s
                                        {
                                            byCntCmdVel  = RESET;
                                            fgSendCmdVel = ON;
                                        }
                                        nArray[0] = PID_PNT_MAIN_DATA;
                                        PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                                    }
                                }
                            }
                            else if(Com.bRccState == ON)
                            {
                                if(Com.bEmerSW == ON)
                                {
                                    fgSendCmdVel = OFF;
                                    nArray[0] = ENABLE;
                                    nArray[1] = ENABLE;
                                    nArray[2] = REQUEST_PNT_MAIN_DATA;
                                    PutMdData(PID_PNT_TQ_OFF, Com.nRMID, nArray);
                                }
                                else
                                {
                                    if(byCntCmdVel < 40) byCntCmdVel++;
                                    else if(byCntCmdVel == 40) //after 2s
                                    {
                                        byCntCmdVel  = RESET;
                                        fgSendCmdVel = ON;
                                    }
                                    nArray[0] = PID_PNT_MAIN_DATA;
                                    PutMdData(PID_REQ_PID_DATA, Com.nRMID, nArray);
                                }
                            }
                        }
                        break;
                    case 9:
                        break;
                    case 10:
                        byCntComStep = 0;
                        break;
                }
            }
            else
            {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else
                {
                    switch(++byCntInitStep)
                    {
                        case 1:
                            nArray[0] = 0;
                            nArray[1] = 0;
                            nArray[2] = 0;
                            nArray[3] = 0;
                            nArray[4] = REQUEST_PNT_MAIN_DATA;
                            PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray);
                            break;
                        case 4:
                            if(Com.nRMID == Com.nIDMDUI) InitSetParam();   //posture initialization in com.cpp
                            InitRobotParam();
                            break;
                        case 7:
                            ResetPosture();                                //reset about odometry's variable
                            break;
                        case 10:
                            InitSetSlowStart();
                            break;
                        case 13:
                            InitSetSlowDown();
                            break;
                        case 15:
                            if(Com.nRMID == MID_MDUI) InitRobotMotDir();
                            Com.fgInitsetting = ON;
                            break;
                    }
                }
            }
        }
        ros::spinOnce();
        r.sleep();
    } 
}
