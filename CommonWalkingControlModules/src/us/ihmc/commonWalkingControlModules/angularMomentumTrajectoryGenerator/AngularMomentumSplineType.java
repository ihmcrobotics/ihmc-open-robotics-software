package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

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
