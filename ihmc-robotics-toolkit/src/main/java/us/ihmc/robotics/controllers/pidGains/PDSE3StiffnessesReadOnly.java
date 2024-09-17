package us.ihmc.robotics.controllers.pidGains;

public interface PDSE3StiffnessesReadOnly
{
   /**
    * Returns the apparent Stiffnesses to be used for the position control.
    *
    * @return the position PD Stiffnesses.
    */
   public abstract PD3DStiffnessesReadOnly getPositionStiffnesses();

   /**
    * Returns the apparent Stiffnesses to be used for the orientation control.
    *
    * @return the orientation PD stiffnesses.
    */
   public abstract PD3DStiffnessesReadOnly getOrientationStiffnesses();

   public default boolean equals(PDSE3StiffnessesReadOnly other)
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
         if (!getPositionStiffnesses().equals(other.getPositionStiffnesses()))
            return false;
         if (!getOrientationStiffnesses().equals(other.getOrientationStiffnesses()))
            return false;
         return true;
      }
   }
}
