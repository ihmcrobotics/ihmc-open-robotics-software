package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public class InverseKinematicsOptimizationSettingsCommand implements InverseKinematicsCommand<InverseKinematicsOptimizationSettingsCommand>
{
   private double jointVelocityWeight = Double.NaN;
   private double jointAccelerationWeight = Double.NaN;

   public void setJointVelocityWeight(double jointVelocityWeight)
   {
      this.jointVelocityWeight = jointVelocityWeight;
   }

   public void setJointAccelerationWeight(double jointAccelerationWeight)
   {
      this.jointAccelerationWeight = jointAccelerationWeight;
   }

   public boolean hasJointVelocityWeight()
   {
      return !Double.isNaN(jointVelocityWeight);
   }

   public boolean hasJointAccelerationWeight()
   {
      return !Double.isNaN(jointAccelerationWeight);
   }

   public double getJointVelocityWeight()
   {
      return jointVelocityWeight;
   }

   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   @Override
   public void set(InverseKinematicsOptimizationSettingsCommand other)
   {
      jointVelocityWeight = other.jointVelocityWeight;
      jointAccelerationWeight = other.jointAccelerationWeight;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }
}
