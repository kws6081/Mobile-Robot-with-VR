#include "md/global.hpp"
#include "md/robot.hpp"
#include "md/main.hpp"
#include "md/com.hpp"
//
int ResetPosture(void)
{
    int byArray[1];

    Log.fgSet           = OFF;
    byArray[0]          = 0;

    RB.dTravelAng       = 0;
    RB.dExTravelAng     = 0;
    RB.dExAng           = 0;

    RB.dWheelTravel[LW] = 0;
    RB.dWheelTravel[RW] = 0;
    RB.dExTravel[0]     = 0;
    RB.dExTravel[1]     = 0;
    RB.dCoord[_X]       = 0;
    RB.dCoord[_Y]       = 0;
    RB.lPosi[_X]        = 0;
    RB.lPosi[_Y]        = 0;
    RB.dAng             = 0;

    PutMdData(PID_POSI_RESET, Com.nRMID, byArray);

    return SUCCESS;
}

int Ang2Ang360(int nAng)
{

    nAng %= 3600;
    if(nAng<0) nAng += 3600;

    return nAng;
}

double RBSpeed2MotRPM(long lSpeed)
{
        if(RB.nDenom==0) return 0;
        return ((double)lSpeed*(long)RB.nNum)/(long)RB.nDenom;
}

double RBAngRatio2RPM(long lAngRatio)
{
    double dAngRatio, dResult;

    if(Com.nDiameter == 0) return 0;

    if(Com.nAngResol == 0)
        dResult = ((double)lAngRatio*(double)Com.nWheelLength*(double)Com.nGearRatio)/(6*(double)Com.nDiameter);
    else if(Com.nAngResol == 1)
    {
        dAngRatio = ((double)lAngRatio)/10;
        dResult = ((double)lAngRatio*(double)Com.nWheelLength*(double)Com.nGearRatio)/(6*(double)Com.nDiameter);
    }

    //printf("%f \n", dResult);
    return dResult;
}

double MotRPM2RBSpeed(long lRPM)
{
        if(RB.nNum==0) return 0;
        return ((double)lRPM*(long)RB.nDenom)/(long)RB.nNum;
}

double MotPosi2RBPosi(long lPosi)
{
        if(RB.dW2RFac==0) return 0;
        return ((double)lPosi*(double)314*(long)Com.nDiameter)/RB.dW2RFac;
}

int MotData2RobotPosture(BYTE byLState, long lLeftRPM, long lLeftPosi, BYTE byRState, long lRightRPM, long lRightPosi)
{
        static long lExRPM;
        long  lAvrRPM;
        double dAng, dDelAvrDisp;
        static double dExDelAvrDisp;
        static BYTE fgExResetAngle;

        switch(Com.fgDirSign)
        {
            case 0:
                lAvrRPM = ((RW_SIGN*lLeftRPM) + (RW_SIGN*lRightRPM))/2;
                if(Abs(lAvrRPM-lExRPM)>MAX_SPEED_DIFF) lAvrRPM = lExRPM;
                lExRPM = lAvrRPM;

                if(lLeftRPM || lRightRPM) RB.fgBusy = 1;
                else RB.fgBusy = 0;

                RB.lSpeed = MotRPM2RBSpeed(lAvrRPM);	// mm/s
                RB.dWheelTravel[LW] = MotPosi2RBPosi(RW_SIGN*lLeftPosi);
                RB.dWheelTravel[RW] = MotPosi2RBPosi(RW_SIGN*lRightPosi);
                break;
            case 1:
                lAvrRPM = ((RW_SIGN*lLeftRPM) + (LW_SIGN*lRightRPM))/2;
                if(Abs(lAvrRPM-lExRPM)>MAX_SPEED_DIFF) lAvrRPM = lExRPM;
                lExRPM = lAvrRPM;

                if(lLeftRPM || lRightRPM) RB.fgBusy = 1;
                else RB.fgBusy = 0;

                RB.lSpeed = MotRPM2RBSpeed(lAvrRPM);	// mm/s
                RB.dWheelTravel[LW] = MotPosi2RBPosi(RW_SIGN*lLeftPosi);
                RB.dWheelTravel[RW] = MotPosi2RBPosi(LW_SIGN*lRightPosi);
                break;
            case 2:
                lAvrRPM = ((LW_SIGN*lLeftRPM) + (RW_SIGN*lRightRPM))/2;
                if(Abs(lAvrRPM-lExRPM)>MAX_SPEED_DIFF) lAvrRPM = lExRPM;
                lExRPM = lAvrRPM;

                if(lLeftRPM || lRightRPM) RB.fgBusy = 1;
                else RB.fgBusy = 0;

                RB.lSpeed = MotRPM2RBSpeed(lAvrRPM);	// mm/s
                RB.dWheelTravel[RW] = MotPosi2RBPosi(LW_SIGN*lLeftPosi);
                RB.dWheelTravel[LW] = MotPosi2RBPosi(RW_SIGN*lRightPosi);
                break;
            case 3:
                lAvrRPM = ((LW_SIGN*lLeftRPM) + (LW_SIGN*lRightRPM))/2;
                if(Abs(lAvrRPM-lExRPM)>MAX_SPEED_DIFF) lAvrRPM = lExRPM;
                lExRPM = lAvrRPM;

                if(lLeftRPM || lRightRPM) RB.fgBusy = 1;
                else RB.fgBusy = 0;

                RB.lSpeed = MotRPM2RBSpeed(lAvrRPM);	// mm/s
                RB.dWheelTravel[LW] = MotPosi2RBPosi(LW_SIGN*lLeftPosi);
                RB.dWheelTravel[RW] = MotPosi2RBPosi(LW_SIGN*lRightPosi);
                break;

        }



        if(Com.nWheelLength)
        {
                RB.dTravelAng = ((RB.dWheelTravel[RW] -
                        RB.dWheelTravel[LW])*(double)18000)/((long)Com.nWheelLength*(double)314);
                dAng = (RB.dWheelTravel[RW] - RB.dWheelTravel[LW])/(long)Com.nWheelLength;

                if(Com.fgResetAngle == 1)
                {
                    if((fgExResetAngle == 0) && (Com.fgResetAngle == 1))
                    {
                        fgExResetAngle  = 1;
                        RB.dExTravelAng = RB.dTravelAng;
                        RB.dExAng       = dAng;
                    }
                }
                else
                    fgExResetAngle  = 0;

        }

        RB.dAng = Ang2Ang360((long)((RB.dTravelAng - RB.dExTravelAng)*10));

        dDelAvrDisp = ((RB.dWheelTravel[LW]-RB.dExTravel[LW]) +
                (RB.dWheelTravel[RW]-RB.dExTravel[RW]))/2;
        if(Abs(dExDelAvrDisp-dDelAvrDisp)>MAX_DISP_DIFF) {
                dDelAvrDisp = dExDelAvrDisp;
        }
        dExDelAvrDisp = dDelAvrDisp;

        RB.dCoord[_X] -= dDelAvrDisp*sin(dAng);
        RB.dCoord[_Y] += dDelAvrDisp*cos(dAng);

        Com.sTheta = RB.dAng;

        Com.lPosi[_X] = (long)RB.dCoord[_X];
        Com.lPosi[_Y] = (long)RB.dCoord[_Y];

        if(Com.fgDirSign == 0)
        {
            RB.nRcvLinearVel  = (((float)lRightRPM-(float)lLeftRPM)*(float)RB.nDenom/(2*(float)RB.nNum));
            RB.nRcvAngularVel = (((float)lRightRPM+(float)lLeftRPM)*(float)3*Com.nDiameter
                                 /((float)Com.nWheelLength*(float)Com.nGearRatio))*10;
        }
        else if(Com.fgDirSign == 1)
        {
            RB.nRcvLinearVel  = -(((float)lRightRPM-(float)lLeftRPM)*(float)RB.nDenom/(2*(float)RB.nNum));
            RB.nRcvAngularVel = -(((float)lRightRPM+(float)lLeftRPM)*(float)3*Com.nDiameter
                                 /((float)Com.nWheelLength*(float)Com.nGearRatio))*10;
        }


        //ROS_INFO("%d \n", RB.nRcvLinearVel);
        RB.dExTravel[LW] = RB.dWheelTravel[LW];
        RB.dExTravel[RW] = RB.dWheelTravel[RW];

        return SUCCESS;
}

int RobotCmd2MotCmd(int nRefSpeed, int nAngRatio)
{
        int nRefRPM, nDelRPM;

        nRefRPM = RBSpeed2MotRPM(nRefSpeed);
        nDelRPM = RBAngRatio2RPM(nAngRatio);

        switch(Com.fgDirSign)
        {
            case 0:
                RB.sRefRPM[0] = nRefRPM - nDelRPM;
                RB.sRefRPM[1] = nRefRPM + nDelRPM;
                break;
            case 1:
                RB.sRefRPM[0] = nRefRPM - nDelRPM;
                RB.sRefRPM[1] = -(nRefRPM + nDelRPM);
                break;
            case 2:
                RB.sRefRPM[0] = -(nRefRPM - nDelRPM);
                RB.sRefRPM[1] = nRefRPM + nDelRPM;
                break;
            case 3:
                RB.sRefRPM[0] = -(nRefRPM - nDelRPM);
                RB.sRefRPM[1] = -(nRefRPM + nDelRPM);
                break;
        }

        return SUCCESS;
}

int InitRobotParam(void)
{
        RB.nPPR         = Com.nHallType*3;
        RB.nNum         = 6000*Com.nGearRatio; // 120,000
        RB.nDenom       = 314*Com.nDiameter;
        RB.dW2RFac      = (double)RB.nPPR*100*(long)Com.nGearRatio;
        RB.nAngRatioDev = Com.nMaxRPM*RPM2DPS;

        return SUCCESS;
}

