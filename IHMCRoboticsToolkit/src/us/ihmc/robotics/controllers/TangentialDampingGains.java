package us.ihmc.robotics.controllers;

public interface TangentialDampingGains
{
   public void set(double kdReductionRatio, double parallelDampingDeadband);

   public double getKdReductionRatio();

   public double getParallelDampingDeadband();
}
