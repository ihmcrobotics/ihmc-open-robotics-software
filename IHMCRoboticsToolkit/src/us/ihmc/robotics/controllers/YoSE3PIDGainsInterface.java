package us.ihmc.robotics.controllers;


public interface YoSE3PIDGainsInterface
{
   public abstract YoPositionPIDGainsInterface getPositionGains();
   
   public abstract YoOrientationPIDGainsInterface getOrientationGains();
}
