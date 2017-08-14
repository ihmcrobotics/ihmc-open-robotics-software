package us.ihmc.robotics.controllers.pidGains;

public interface PIDSE3Gains
{
   public abstract PID3DGains getPositionGains();

   public abstract PID3DGains getOrientationGains();

   public default void setOrientationGains(PID3DGainsReadOnly orientationGains)
   {
      getOrientationGains().set(orientationGains);
   }

   public default void setPositionGains(PID3DGainsReadOnly positionGains)
   {
      getPositionGains().set(positionGains);
   }

   public default void set(PIDSE3Gains gains)
   {
      setOrientationGains(gains.getOrientationGains());
      setPositionGains(gains.getPositionGains());
   }
}