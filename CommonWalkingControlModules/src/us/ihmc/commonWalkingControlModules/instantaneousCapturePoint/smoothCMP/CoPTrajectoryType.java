package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

public enum CoPTrajectoryType
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
