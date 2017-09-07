package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public class LegConfigurationGains
{
   private double jointSpaceKp = Double.NaN;
   private double jointSpaceKd = Double.NaN;
   private double actuatorSpaceKp = Double.NaN;
   private double actuatorSpaceKd = Double.NaN;

   private boolean blendPositionError = false;
   private boolean blendVelocityError = false;

   private boolean useActuatorSpacePositionControl = false;
   private boolean useActuatorSpaceVelocityControl = false;

   private double maxPositionBlendingFactor = 0.0;
   private double maxVelocityBlendingFactor = 0.0;

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

   public void setMaxPositionBlendingFactor(double factor)
   {
      maxPositionBlendingFactor = factor;
   }

   public void setMaxVelocityBlendingFactor(double factor)
   {
      maxVelocityBlendingFactor = factor;
   }

   public void setBlendPositionError(boolean blend)
   {
      blendPositionError = blend;
   }

   public void setBlendVelocityError(boolean blend)
   {
      blendVelocityError = blend;
   }

   public void setUseActuatorSpacePositionControl(boolean useActuatorSpacePositionControl)
   {
      this.useActuatorSpacePositionControl = useActuatorSpacePositionControl;
   }

   public void setUseActuatorSpaceVelocityControl(boolean useActuatorSpaceVelocityControl)
   {
      this.useActuatorSpaceVelocityControl = useActuatorSpaceVelocityControl;
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

   public double getMaxPositionBlendingFactor()
   {
      return maxPositionBlendingFactor;
   }

   public double getMaxVelocityBlendingFactor()
   {
      return maxVelocityBlendingFactor;
   }

   public boolean getBlendPositionError()
   {
      return blendPositionError;
   }

   public boolean getBlendVelocityError()
   {
      return blendVelocityError;
   }

   public boolean getUseActuatorSpacePositionControl()
   {
      return useActuatorSpacePositionControl;
   }

   public boolean getUseActuatorSpaceVelocityControl()
   {
      return useActuatorSpaceVelocityControl;
   }
}
