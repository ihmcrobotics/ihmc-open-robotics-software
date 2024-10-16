package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import us.ihmc.wholeBodyControlCore.pidGains.GainCalculator;
import us.ihmc.wholeBodyControlCore.pidGains.PDGainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.PDGainsReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class PDGains implements PDGainsBasics
{
   private double kp;
   private double kd;
   private double zeta;
   private double maximumFeedback = Double.POSITIVE_INFINITY;
   private double maximumFeedbackRate = Double.POSITIVE_INFINITY;
   private double positionDeadband = 0.0;

   private final DoubleProvider maximumFeedbackProvider = () -> maximumFeedback;
   private final DoubleProvider maximumFeedbackRateProvider = () -> maximumFeedbackRate;

   @Override
   public double getKp()
   {
      return kp;
   }

   @Override
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

   @Override
   public void setKd(double kd)
   {
      this.kd = kd;
      zeta = GainCalculator.computeDampingRatio(kp, kd);
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

   @Override
   public void setMaximumFeedback(double maximumFeedback)
   {
      this.maximumFeedback = maximumFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   @Override
   public void setMaximumFeedbackRate(double maximumFeedbackRate)
   {
      this.maximumFeedbackRate = maximumFeedbackRate;
   }

   @Override
   public double getPositionDeadband()
   {
      return positionDeadband;
   }

   @Override
   public void setPositionDeadband(double positionDeadband)
   {
      this.positionDeadband = positionDeadband;
   }

   @Override
   public DoubleProvider getMaximumFeedbackProvider()
   {
      return maximumFeedbackProvider;
   }

   @Override
   public DoubleProvider getMaximumFeedbackRateProvider()
   {
      return maximumFeedbackRateProvider;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PDGains)
      {
         PDGains other = (PDGains) object;
         if (Double.compare(zeta, other.zeta) != 0)
            return false;
         if (!PDGainsBasics.super.equals(other))
            return false;
         return true;
      }
      else if (object instanceof PDGainsReadOnly)
      {
         return PDGainsBasics.super.equals((PDGainsReadOnly) object);
      }
      else
      {
         return false;
      }
   }
}
