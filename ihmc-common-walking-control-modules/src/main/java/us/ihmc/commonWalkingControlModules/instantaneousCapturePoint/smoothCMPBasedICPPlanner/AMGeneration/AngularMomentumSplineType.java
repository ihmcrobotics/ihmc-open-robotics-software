package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.AMGeneration;

public enum AngularMomentumSplineType
{
   LINEAR;
   
   public String toString()
   {
      switch (this)
      {
      default: return "Linear";
      }
   }
   
   public int getNumberOfCoefficients()
   {
      switch (this)
      {
      default: return 2;
      }
   }
}
