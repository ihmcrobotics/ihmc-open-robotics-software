package us.ihmc.commonWalkingControlModules.controlModules;

public class LeeGoswamiForceOptimizerNative
{
   public native double[] computeForces(double[] phi, double[] xi, double epsilonf);
}
