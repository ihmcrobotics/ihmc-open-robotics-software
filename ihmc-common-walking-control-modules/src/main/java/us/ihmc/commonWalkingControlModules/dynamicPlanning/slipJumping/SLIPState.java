package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

public enum SLIPState
{
   STANCE, FLIGHT;

   public final static int stateVectorSize = 12;
   public final static int controlVectorSize = 9;
   public final static int constantVectorSize = 2;

   public final static int x = 0;
   public final static int y = 1;
   public final static int z = 2;
   public final static int thetaX = 3;
   public final static int thetaY = 4;
   public final static int thetaZ = 5;
   public final static int xDot = 6;
   public final static int yDot = 7;
   public final static int zDot = 8;
   public final static int thetaXDot = 9;
   public final static int thetaYDot = 10;
   public final static int thetaZDot = 11;

   public final static int fx = 0;
   public final static int fy = 1;
   public final static int fz = 2;
   public final static int tauX = 3;
   public final static int tauY = 4;
   public final static int tauZ = 5;
   public final static int xF = 6;
   public final static int yF = 7;
   public final static int k = 8;

   public final static int zF = 0;
   public final static int nominalLength = 1;
}
