package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public class LegConfigurationGains
{
   private double jointSpaceKp = Double.NaN;
   private double jointSpaceKd = Double.NaN;

   public void setJointSpaceKp(double kp)
   {
      jointSpaceKp = kp;
   }

   public void setJointSpaceKd(double kd)
   {
      jointSpaceKd = kd;
   }

   public double getJointSpaceKp()
   {
      return jointSpaceKp;
   }

   public double getJointSpaceKd()
   {
      return jointSpaceKd;
   }
}
