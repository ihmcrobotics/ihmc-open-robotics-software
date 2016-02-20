package us.ihmc.robotics.controllers;


public interface YoSE3PIDGains
{
   public abstract YoPositionPIDGainsInterface getPositionGains();
   
   public abstract YoOrientationPIDGainsInterface getOrientationGains();
}
