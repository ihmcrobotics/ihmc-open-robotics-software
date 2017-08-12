package us.ihmc.robotics.controllers.pidGains;

public interface PIDSE3Gains
{
   public abstract void setOrientationGains(PID3DGainsReadOnly orientationGains);

   public abstract void setPositionGains(PID3DGainsReadOnly positionGains);

   public abstract PID3DGains getPositionGains();

   public abstract PID3DGains getOrientationGains();

   public default void set(PIDSE3Gains gains)
   {
      setOrientationGains(gains.getOrientationGains());
      setPositionGains(gains.getPositionGains());
   }
}