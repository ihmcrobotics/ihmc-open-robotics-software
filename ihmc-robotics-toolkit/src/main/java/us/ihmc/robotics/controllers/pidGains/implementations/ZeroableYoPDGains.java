package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * ZeroableYoPDGains holds the gains as yovariables.
 * 
 * It does not contain a zeta, so if Kp and Kd are set to zero, it doesn't get a NaN value for zeta, which could be dangerous.
 * 
 * @author jesper
 *
 */
public class ZeroableYoPDGains implements PDGainsReadOnly
{
   private final YoDouble kp;
   private final YoDouble kd;
   private final YoDouble maximumFeedback;
   private final YoDouble maximumFeedbackRate;
   private final YoDouble positionDeadband;

   public ZeroableYoPDGains(String suffix, YoRegistry registry)
   {
      kp = new YoDouble("kp" + suffix, registry);
      kd = new YoDouble("kd" + suffix, registry);

      maximumFeedback = new YoDouble("maximumFeedback" + suffix, registry);
      maximumFeedbackRate = new YoDouble("maximumFeedbackRate" + suffix, registry);

      positionDeadband = new YoDouble("positionDeadband" + suffix, registry);

      maximumFeedback.set(Double.POSITIVE_INFINITY);
      maximumFeedbackRate.set(Double.POSITIVE_INFINITY);
   }

   public void setKp(double kp)
   {
      this.kp.set(kp);
   }

   public void setKd(double kd)
   {
      this.kd.set(kd);
   }


   public void setMaximumFeedback(double maxFeedback)
   {
      this.maximumFeedback.set(maxFeedback);
   }

   public void setMaximumFeedbackRate(double maxFeedbackRate)
   {
      this.maximumFeedbackRate.set(maxFeedbackRate);
   }

   public void setMaximumFeedbackAndMaximumFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      maximumFeedback.set(maxFeedback);
      maximumFeedbackRate.set(maxFeedbackRate);
   }

   public void setPositionDeadband(double deadband)
   {
      positionDeadband.set(deadband);
   }

   @Override
   public double getKp()
   {
      return kp.getDoubleValue();
   }

   @Override
   public double getKd()
   {
      return kd.getDoubleValue();
   }

   @Override
   public double getMaximumFeedback()
   {
      return maximumFeedback.getDoubleValue();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate.getDoubleValue();
   }

   @Override
   public double getPositionDeadband()
   {
      return positionDeadband.getDoubleValue();
   }

   public YoDouble getYoKp()
   {
      return kp;
   }

   public YoDouble getYoKd()
   {
      return kd;
   }

   public YoDouble getYoMaximumFeedback()
   {
      return maximumFeedback;
   }

   public YoDouble getYoMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   public YoDouble getYoPositionDeadband()
   {
      return positionDeadband;
   }

   public void set(ZeroableYoPDGains other)
   {
      this.kp.set(other.kp.getDoubleValue());
      this.kd.set(other.kd.getDoubleValue());
      this.maximumFeedback.set(other.maximumFeedback.getDoubleValue());
      this.maximumFeedbackRate.set(other.maximumFeedbackRate.getDoubleValue());
      this.positionDeadband.set(other.positionDeadband.getDoubleValue());
   }

   public void set(PDGainsReadOnly pdGains)
   {
      setKp(pdGains.getKp());
      setKd(pdGains.getKd());
      setMaximumFeedback(pdGains.getMaximumFeedback());
      setMaximumFeedbackRate(pdGains.getMaximumFeedbackRate());
      setPositionDeadband(pdGains.getPositionDeadband());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PDGainsReadOnly)
         return PDGainsReadOnly.super.equals((PDGainsReadOnly) object);
      else
         return false;
   }
}