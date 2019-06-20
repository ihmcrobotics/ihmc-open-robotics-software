package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;

public class PDGains implements PDGainsReadOnly, Settable<PDGains>
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

   @Override
   public void set(PDGains other)
   {
      set((PDGainsReadOnly) other);
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

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PDGains)
      {
         PDGains other = (PDGains) object;
         if (Double.compare(zeta, other.zeta) != 0)
            return false;
         if (!PDGainsReadOnly.super.equals(other))
            return false;
         return true;
      }
      else if (object instanceof PDGainsReadOnly)
      {
         return PDGainsReadOnly.super.equals((PDGainsReadOnly) object);
      }
      else
      {
         return false;
      }
   }
}
