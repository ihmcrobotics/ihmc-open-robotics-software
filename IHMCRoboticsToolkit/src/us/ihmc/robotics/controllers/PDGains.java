package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.GainCalculator;

public class PDGains
{
   private final String name;

   private double kp;
   private double kd;
   private double zeta;
   private double maximumOutput;
   private double maximumFeedback;
   private double maximumFeedbackRate;
   private double positionDeadband;

   public PDGains()
   {
      this("");
   }

   public PDGains(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public double getKp()
   {
      return kp;
   }

   public void setKp(double kp)
   {
      this.kp = kp;
      kd = GainCalculator.computeDerivativeGain(kp, zeta);
   }

   public double getZeta()
   {
      return zeta;
   }

   public void setZeta(double zeta)
   {
      this.zeta = zeta;
      kd = GainCalculator.computeDerivativeGain(kp, zeta);
   }

   public double getKd()
   {
      return kd;
   }

   public double getMaximumOutput()
   {
      return maximumOutput;
   }

   public void setMaximumOutput(double maximumOutput)
   {
      this.maximumOutput = maximumOutput;
   }

   public double getMaximumFeedback()
   {
      return maximumFeedback;
   }

   public void setMaximumFeedback(double maximumFeedback)
   {
      this.maximumFeedback = maximumFeedback;
   }

   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   public void setMaximumFeedbackRate(double maximumFeedbackRate)
   {
      this.maximumFeedbackRate = maximumFeedbackRate;
   }

   public double getPositionDeadband()
   {
      return positionDeadband;
   }

   public void setPositionDeadband(double positionDeadband)
   {
      this.positionDeadband = positionDeadband;
   }
}
