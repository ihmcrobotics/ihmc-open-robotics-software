package us.ihmc.robotics.controllers;


public interface YoSE3PIDGains
{
   public abstract YoPositionPIDGains getPositionGains();
   
   public abstract YoOrientationPIDGains getOrientationGains();
}
