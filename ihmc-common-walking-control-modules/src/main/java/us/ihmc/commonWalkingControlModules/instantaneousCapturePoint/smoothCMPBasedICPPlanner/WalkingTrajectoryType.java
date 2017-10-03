package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

public enum WalkingTrajectoryType
{
   SWING, TRANSFER;
   
   @Override
   public String toString()
   {
      switch (this)
      {
      case SWING: return "Swing";
      default: return "Transfer";
      }
   }
}
