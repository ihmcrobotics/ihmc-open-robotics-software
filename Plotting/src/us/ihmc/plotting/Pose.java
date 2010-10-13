package us.ihmc.plotting;

import java.io.Serializable;

public class Pose extends Coordinate implements Serializable
{
   private static final long serialVersionUID = 3781748456290298303L;
   private double roll;
   private double pitch;
   private double yaw;

   public Pose(double x, double y, double yaw, int unit)
   {
      super(x, y, unit);
      this.yaw = yaw;
   }

   public Pose(double roll, double pitch, double yaw)
   {
      super();
      this.roll = roll;
      this.pitch = pitch;
      this.yaw = yaw;
   }

   public double getRoll()
   {
      return roll;
   }

   public double getPitch()
   {
      return pitch;
   }

   public double getYaw()
   {
      return yaw;
   }

   public void setRoll(double roll)
   {
      this.roll = roll;
   }

   public void setPitch(double pitch)
   {
      this.pitch = pitch;
   }

   public void setYaw(double yaw)
   {
      this.yaw = yaw;
   }

   public boolean equals(Pose pose)
   {
      if (!super.equals(pose))
         return false;
      if (roll != pose.getRoll())
         return false;
      if (pitch != pose.getPitch())
         return false;
      if (Math.abs(yaw - pose.getYaw()) > 0.1)
         return false;

      return true;
   }

   public String toString()
   {
      return "(" + getX() + ", " + getY() + ", " + getZ() + ", " + getRoll() + ", " + getPitch() + ", " + getYaw() + ")";
   }
}
