package us.ihmc.robotics.controllers.pidGains;

/**
 * Interface for SE3 PID gains consisting of two PID gains in 3D. One for the
 * position control and one for the orientation control.
 */
public interface PIDSE3Gains
{
   /**
    * Returns the gains to be used for the position control.
    *
    * @return the position PID gains.
    */
   public abstract PID3DGains getPositionGains();

   /**
    * Returns the gains to be used for the orientation control.
    *
    * @return the orientation PID gains.
    */
   public abstract PID3DGains getOrientationGains();

   /**
    * Sets the position gains to the provided PID gains.
    *
    * @param positionGains the new gains for position control.
    */
   public default void setPositionGains(PID3DGainsReadOnly positionGains)
   {
      getPositionGains().set(positionGains);
   }

   /**
    * Sets the orientation gains to the provided PID gains.
    *
    * @param orientationGains the new gains for orientation control.
    */
   public default void setOrientationGains(PID3DGainsReadOnly orientationGains)
   {
      getOrientationGains().set(orientationGains);
   }

   /**
    * Copies the gains and parameters from the provided {@link PIDSE3Gains}
    * parameters into this.
    *
    * @param other the new gains.
    */
   public default void set(PIDSE3Gains other)
   {
      setOrientationGains(other.getOrientationGains());
      setPositionGains(other.getPositionGains());
   }
}