package us.ihmc.robotics.controllers;

public class SimplePDGainsHolder implements PDGainsInterface
{
   private double kp = 0.0;
   private double kd = 0.0;
   private double maxAcceleration = Double.POSITIVE_INFINITY;
   private double maxJerk = Double.POSITIVE_INFINITY;

   public SimplePDGainsHolder()
   {
   }

   public void set(PDGainsInterface other)
   {
      setKp(other.getKp());
      setKd(other.getKd());
      setMaxAcceleration(other.getMaximumAcceleration());
      setMaxJerk(other.getMaximumJerk());
   }

   public void set(double kp, double kd, double maxAcceleration, double maxJerk)
   {
      setKp(kp);
      setKd(kd);
      setMaxAcceleration(maxAcceleration);
      setMaxJerk(maxJerk);
   }

   public void setKp(double kp)
   {
      this.kp = kp;
   }

   public void setKd(double kd)
   {
      this.kd = kd;
   }

   public void setMaxAcceleration(double maxAcceleration)
   {
      this.maxAcceleration = maxAcceleration;
   }

   public void setMaxJerk(double maxJerk)
   {
      this.maxJerk = maxJerk;
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
   public double getMaximumAcceleration()
   {
      return maxAcceleration;
   }

   @Override
   public double getMaximumJerk()
   {
      return maxJerk;
   }
}
