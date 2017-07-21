package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public class LegConfigurationGains
{
   private double jointSpaceKp = Double.NaN;
   private double jointSpaceKd = Double.NaN;
   private double actuatorSpaceKp = Double.NaN;
   private double actuatorSpaceKd = Double.NaN;

   private boolean blendPositionError = false;
   private boolean blendVelocityError = false;

   private double maxBlendingFactor = 0.0;

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

   public void setMaxBlendingFactor(double factor)
   {
      maxBlendingFactor = factor;
   }

   public void setBlendPositionError(boolean blend)
   {
      blendPositionError = blend;
   }

   public void setBlendVelocityError(boolean blend)
   {
      blendVelocityError = blend;
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

   public double getMaxBlendingFactor()
   {
      return maxBlendingFactor;
   }

   public boolean getBlendPositionError()
   {
      return blendPositionError;
   }

   public boolean getBlendVelocityError()
   {
      return blendVelocityError;
   }
}
