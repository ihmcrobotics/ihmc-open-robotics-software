package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public class InverseKinematicsOptimizationSettingsCommand implements InverseKinematicsCommand<InverseKinematicsOptimizationSettingsCommand>
{
   public enum JointVelocityLimitMode
   {
      ENABLED, DISABLED
   };

   private double jointVelocityWeight = Double.NaN;
   private double jointAccelerationWeight = Double.NaN;
   public JointVelocityLimitMode jointVelocityLimitMode = null;

   /**
    * Sets the weight specifying how much high joint velocity values should be penalized in the
    * optimization problem.
    * <p>
    * A non-zero positive value should be used to ensure the Hessian matrix in the optimization is
    * invertible. It is should preferably be above {@code 1.0e-8}.
    * </p>
    *
    * @param jointVelocityWeight the weight to use for joint velocity regularization.
    */
   public void setJointVelocityWeight(double jointVelocityWeight)
   {
      this.jointVelocityWeight = jointVelocityWeight;
   }

   /**
    * Sets the weight specifying how much high joint acceleration values should be penalized in the
    * optimization problem.
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight helps
    * to improve smoothness of the resulting motions. A high value will cause the system to become too
    * 'springy'.
    * </p>
    *
    * @param jointAccelerationWeight the weight to use for joint acceleration regularization.
    */
   public void setJointAccelerationWeight(double jointAccelerationWeight)
   {
      this.jointAccelerationWeight = jointAccelerationWeight;
   }

   /**
    * Sets whether the joint velocity limits should be considered or not.
    * 
    * @param jointVelocityLimitMode the new value for considering joint velocity limits.
    */
   public void setJointVelocityLimitMode(JointVelocityLimitMode jointVelocityLimitMode)
   {
      this.jointVelocityLimitMode = jointVelocityLimitMode;
   }

   /**
    * Whether this command holds onto a new value for {@code jointVelocityWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasJointVelocityWeight()
   {
      return !Double.isNaN(jointVelocityWeight);
   }

   /**
    * Whether this command holds onto a new value for {@code jointAccelerationWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasJointAccelerationWeight()
   {
      return !Double.isNaN(jointAccelerationWeight);
   }

   /**
    * Whether this command holds onto a new value for {@code jointVelocityLimitMode} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hashJointVelocityLimitMode()
   {
      return jointVelocityLimitMode != null;
   }

   /**
    * Gets the value for {@code jointVelocityWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code jointVelocityWeight}.
    * @see #hasJointVelocityWeight()
    * @see #setJointVelocityWeight(double)
    */
   public double getJointVelocityWeight()
   {
      return jointVelocityWeight;
   }

   /**
    * Gets the value for {@code jointAccelerationWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code jointAccelerationWeight}.
    * @see #hasJointAccelerationWeight()
    * @see #setJointAccelerationWeight(double)
    */
   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   /**
    * Gets the value for {@code jointVelocityLimitMode}.
    * <p>
    * It is equal to {@code null} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code jointVelocityLimitMode}.
    * @see #hashJointVelocityLimitMode()
    * @see #setJointVelocityLimitMode(JointVelocityLimitMode)
    */
   public JointVelocityLimitMode getJointVelocityLimitMode()
   {
      return jointVelocityLimitMode;
   }

   @Override
   public void set(InverseKinematicsOptimizationSettingsCommand other)
   {
      jointVelocityWeight = other.jointVelocityWeight;
      jointAccelerationWeight = other.jointAccelerationWeight;
      jointVelocityLimitMode = other.jointVelocityLimitMode;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof InverseKinematicsOptimizationSettingsCommand)
      {
         InverseKinematicsOptimizationSettingsCommand other = (InverseKinematicsOptimizationSettingsCommand) object;

         if (Double.compare(jointVelocityWeight, other.jointVelocityWeight) != 0)
            return false;
         if (Double.compare(jointAccelerationWeight, other.jointAccelerationWeight) != 0)
            return false;
         if (jointVelocityLimitMode != other.jointVelocityLimitMode)
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": qd weight: " + jointVelocityWeight + ", qdd weight: " + jointAccelerationWeight + ", qd limits: "
            + jointVelocityLimitMode;
   }
}
