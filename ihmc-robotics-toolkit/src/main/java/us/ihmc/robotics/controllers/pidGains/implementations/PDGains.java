package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;

public class PDGains implements PDGainsReadOnly
{
   private double kp;
   private double kd;
   private double zeta;
   private double maximumFeedback;
   private double maximumFeedbackRate;
   private double positionDeadband;

   @Override
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

   public void setKd(double kd)
   {
      this.kd = kd;
      zeta = GainCalculator.computeDampingRatio(kp, kd);
   }

   public void set(PDGainsReadOnly other)
   {
      setKp(other.getKp());
      setKd(other.getKd());
      setMaximumFeedback(other.getMaximumFeedback());
      setMaximumFeedbackRate(other.getMaximumFeedbackRate());
      setPositionDeadband(other.getPositionDeadband());
   }

   public void set(double kp, double kd, double maxFeedback, double maxFeedbackRate)
   {
      set(kp, kd, maxFeedback, maxFeedbackRate, 0.0);
   }

   public void set(double kp, double kd, double maxFeedback, double maxFeedbackRate, double positionDeadband)
   {
      setKp(kp);
      setKd(kd);
      setMaximumFeedback(maxFeedback);
      setMaximumFeedbackRate(maxFeedbackRate);
      setPositionDeadband(positionDeadband);
   }

   @Override
   public double getKd()
   {
      return kd;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maximumFeedback;
   }

   public void setMaximumFeedback(double maximumFeedback)
   {
      this.maximumFeedback = maximumFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   public void setMaximumFeedbackRate(double maximumFeedbackRate)
   {
      this.maximumFeedbackRate = maximumFeedbackRate;
   }

   @Override
   public double getPositionDeadband()
   {
      return positionDeadband;
   }

   public void setPositionDeadband(double positionDeadband)
   {
      this.positionDeadband = positionDeadband;
   }
}
