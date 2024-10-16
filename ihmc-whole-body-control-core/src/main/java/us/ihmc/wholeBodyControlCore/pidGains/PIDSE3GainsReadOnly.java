package us.ihmc.wholeBodyControlCore.pidGains;

public interface PIDSE3GainsReadOnly
{
   /**
    * Returns the gains to be used for the position control.
    *
    * @return the position PID gains.
    */
   PID3DGainsReadOnly getPositionGains();

   /**
    * Returns the gains to be used for the orientation control.
    *
    * @return the orientation PID gains.
    */
   PID3DGainsReadOnly getOrientationGains();

   default boolean equals(PIDSE3GainsReadOnly other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (!getPositionGains().equals(other.getPositionGains()))
            return false;
         if (!getOrientationGains().equals(other.getOrientationGains()))
            return false;
         return true;
      }
   }
}
