package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

public enum SLIPState
{
   STANCE, FLIGHT;

   final static int stateVectorSize = 12;
   final static int controlVectorSize = 9;

   final static int x = 0;
   final static int y = 1;
   final static int z = 2;
   final static int thetaX = 3;
   final static int thetaY = 4;
   final static int thetaZ = 5;
   final static int xDot = 6;
   final static int yDot = 7;
   final static int zDot = 8;
   final static int thetaXDot = 9;
   final static int thetaYDot = 10;
   final static int thetaZDot = 11;

   final static int fx = 0;
   final static int fy = 1;
   final static int fz = 2;
   final static int tauX = 3;
   final static int tauY = 4;
   final static int tauZ = 5;
   final static int xF = 6;
   final static int yF = 7;
   final static int k = 8;
}
