package us.ihmc.robotics.controllers;

public interface SE3PIDGainsInterface
{
   public abstract void set(SE3PIDGainsInterface gains);

   public abstract void set(OrientationPIDGainsInterface orientationGains);

   public abstract void set(PositionPIDGainsInterface positionGains);

   public abstract PositionPIDGainsInterface getPositionGains();
   
   public abstract OrientationPIDGainsInterface getOrientationGains();
}