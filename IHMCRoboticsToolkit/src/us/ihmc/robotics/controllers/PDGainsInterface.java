package us.ihmc.robotics.controllers;

public interface PDGainsInterface
{
   public abstract double getKp();

   public abstract double getKd();

   public abstract double getMaximumFeedback();

   public abstract double getMaximumFeedbackRate();
}