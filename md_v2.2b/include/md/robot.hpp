#define MY_PI		  314	// 3.14*100
#define RPM2DPS		  6
#define MAX_SPEED_DIFF	  1000	// rpm
#define MAX_DISP_DIFF	  200	// 20cm
#define LW_SIGN           -1
#define RW_SIGN           1

typedef struct {
    int nBaudrate;
    int nRcvBaudrate;
    BYTE byRcvBaudrateType;
    int nDiameter;
    int nWheelLength;
    int nGearRatio;
    int nDenom, nNum;
    double dW2RFac;
    int nPPR;
    int nVoltIn;
    long lRefSpeed;
    long lRefAngRatio;
    short sRefRPM[2];
    double dAng;


    int nAng;
    long lSpeed;
    double dWheelTravel[2];
    double dExTravel[2];
    double dCoord[2];
    double dTravelAng;
    long lPosi[2];
    BYTE fgBusy;

    short sVisionSpeed, sVisionAngSpeed;

    long lLwSign, lRwSign;
    int fgDirSign;

    int nGapX, nGapY, nGapTheta;

    int nRcvLinearVel, nRcvAngularVel;

    int nAngRatioDev;

    double dExTravelAng, dExAng;

}TableOfRobotControlRegister;
extern TableOfRobotControlRegister RB;

extern double RBSpeed2MotRPM(long lSpeed);
extern double RBAngRatio2RPM(long lAngRatio);
extern double MotRPM2RBSpeed(long lRPM);
extern double MotPosi2RBPosi(long lPosi);
extern int MotData2RobotPosture(BYTE byLState, long lLeftRPM, long lLeftPosi,
                                BYTE byRState, long lRightRPM, long lRightPosi);
extern int RobotCmd2MotCmd(int nRefSpeed, int nAngRatio);
extern int InitRobotParam(void);
extern int ResetPosture(void);
