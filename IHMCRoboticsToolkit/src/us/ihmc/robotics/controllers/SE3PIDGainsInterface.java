package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.OrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.pidGains.PositionPIDGainsInterface;

public interface SE3PIDGainsInterface
{
   public abstract void set(SE3PIDGainsInterface gains);

   public abstract void set(OrientationPIDGainsInterface orientationGains);

   public abstract void set(PositionPIDGainsInterface positionGains);

   public abstract PositionPIDGainsInterface getPositionGains();
   
   public abstract OrientationPIDGainsInterface getOrientationGains();
}