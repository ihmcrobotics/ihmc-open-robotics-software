package us.ihmc.robotics.controllers;


public interface YoPlanarSE3PIDGainsInterface extends SE3PIDGainsInterface
{
   @Override
   public abstract YoPlanarPositionPIDGainsInterface getPositionGains();
   
   @Override
   public abstract YoPlanarOrientationPIDGainsInterface getOrientationGains();
}
