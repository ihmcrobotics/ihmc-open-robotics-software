package us.ihmc.trajectoryOptimization.lipm;

public class DDPTools
{
   private static double computeDeltaT(double trajectoryLength, double nominalDeltaT)
   {
      int numberOfTimeSteps = (int) Math.floor(trajectoryLength / nominalDeltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

}
