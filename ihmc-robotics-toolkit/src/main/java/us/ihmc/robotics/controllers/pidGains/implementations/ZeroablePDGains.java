package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;

/**
 * ZeroablePDGains holds the gains as yovariables.
 * 
 * It does not contain a zeta, so if Kp and Kd are set to zero, it doesn't get a NaN value for zeta, which could be dangerous.
 * 
 * @author jesper
 *
 */
public class ZeroablePDGains implements PDGainsReadOnly, Settable<ZeroablePDGains>
{
   private double kp;
   private double kd;
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
   }

   public void setKd(double kd)
   {
      this.kd = kd;
   }

   @Override
   public void set(ZeroablePDGains other)
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
      if (object instanceof PDGainsReadOnly)
      {
         return PDGainsReadOnly.super.equals((PDGainsReadOnly) object);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "ZeroablePDGains [kp=" + kp + ", kd=" + kd + ", maximumFeedback=" + maximumFeedback + ", maximumFeedbackRate=" + maximumFeedbackRate
            + ", positionDeadband=" + positionDeadband + "]";
   }
   
   
}
