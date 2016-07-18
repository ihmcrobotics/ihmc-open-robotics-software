package us.ihmc.robotics.controllers;

public class SimplePDGainsHolder implements PDGainsInterface
{
   private double kp = 0.0;
   private double kd = 0.0;
   private double maxFeedback = Double.POSITIVE_INFINITY;
   private double maxFeedbackRate = Double.POSITIVE_INFINITY;

   public SimplePDGainsHolder()
   {
   }

   public void set(PDGainsInterface other)
   {
      setKp(other.getKp());
      setKd(other.getKd());
      setMaxFeedback(other.getMaximumFeedback());
      setMaxFeedbackRate(other.getMaximumFeedbackRate());
   }

   public void set(double kp, double kd, double maxFeedback, double maxFeedbackRate)
   {
      setKp(kp);
      setKd(kd);
      setMaxFeedback(maxFeedback);
      setMaxFeedbackRate(maxFeedbackRate);
   }

   public void setKp(double kp)
   {
      this.kp = kp;
   }

   public void setKd(double kd)
   {
      this.kd = kd;
   }

   public void setMaxFeedback(double maxFeedback)
   {
      this.maxFeedback = maxFeedback;
   }

   public void setMaxFeedbackRate(double maxFeedbackRate)
   {
      this.maxFeedbackRate = maxFeedbackRate;
   }

   @Override
   public double getKp()
   {
      return kp;
   }

   @Override
   public double getKd()
   {
      return kd;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maxFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maxFeedbackRate;
   }
}
