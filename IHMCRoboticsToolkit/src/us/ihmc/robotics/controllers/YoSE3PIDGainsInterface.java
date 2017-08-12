package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;

public interface YoSE3PIDGainsInterface extends PIDSE3Gains
{
   @Override
   public abstract YoPositionPIDGainsInterface getPositionGains();
   
   @Override
   public abstract YoOrientationPIDGainsInterface getOrientationGains();
}
