package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public class LegConfigurationGains
{
   private double jointSpaceKp;
   private double jointSpaceKd;
   private double actuatorSpaceKp;
   private double actuatorSpaceKd;

   public void setJointSpaceKp(double kp)
   {
      jointSpaceKp = kp;
   }

   public void setJointSpaceKd(double kd)
   {
      jointSpaceKd = kd;
   }

   public void setActuatorSpaceKp(double kp)
   {
      actuatorSpaceKp = kp;
   }

   public void setActuatorSpaceKd(double kd)
   {
      actuatorSpaceKd = kd;
   }

   public double getJointSpaceKp()
   {
      return jointSpaceKp;
   }

   public double getJointSpaceKd()
   {
      return jointSpaceKd;
   }

   public double getActuatorSpaceKp()
   {
      return actuatorSpaceKp;
   }

   public double getActuatorSpaceKd()
   {
      return actuatorSpaceKd;
   }
}
