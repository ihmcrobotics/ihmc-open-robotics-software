package us.ihmc.robotics.controllers;


public interface YoSE3PIDGainsInterface extends SE3PIDGainsInterface
{
   @Override
   public abstract YoPositionPIDGainsInterface getPositionGains();
   
   @Override
   public abstract YoOrientationPIDGainsInterface getOrientationGains();
}
