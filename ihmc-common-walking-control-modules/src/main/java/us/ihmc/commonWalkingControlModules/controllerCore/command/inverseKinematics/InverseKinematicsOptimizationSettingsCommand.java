package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class InverseKinematicsOptimizationSettingsCommand implements InverseKinematicsCommand<InverseKinematicsOptimizationSettingsCommand>
{
   public enum ActivationState
   {
      ENABLED, DISABLED
   };

   private int commandId;
   private double jointVelocityWeight = Double.NaN;
   private double jointAccelerationWeight = Double.NaN;
   private double jointTorqueWeight = Double.NaN;
   private ActivationState jointVelocityLimitMode = null;
   private ActivationState computeJointTorques = null;
   private final List<OneDoFJointBasics> jointsToDeactivate = new ArrayList<>();
   private final List<OneDoFJointBasics> jointsToActivate = new ArrayList<>();

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
    * Sets the weight specifying how much high joint torque values should be penalized in the
    * optimization problem.
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight helps
    * to improve smoothness of the resulting motions. A high value will cause the system to become too
    * 'springy'.
    * </p>
    *
    * @param jointTorqueWeight the weight to use for joint torque regularization.
    */
   public void setJointTorqueWeight(double jointTorqueWeight)
   {
      this.jointTorqueWeight = jointTorqueWeight;
   }

   /**
    * Sets whether the joint velocity limits should be considered or not.
    * 
    * @param jointVelocityLimitMode the new value for considering joint velocity limits.
    */
   public void setJointVelocityLimitMode(ActivationState jointVelocityLimitMode)
   {
      this.jointVelocityLimitMode = jointVelocityLimitMode;
   }

   /**
    * Sets whether the joint torques should be computed or not.
    *
    * @param computeJointTorques the new value for whether to compute the joint torques
    */
   public void setComputeJointTorques(ActivationState computeJointTorques)
   {
      this.computeJointTorques = computeJointTorques;
   }

   /**
    * Activates a joint such that it is assumed to be controllable and its acceleration should be
    * solved by the QP solver.
    * <p>
    * If the joint was already active, nothing happens.
    * </p>
    *
    * @param jointToActivate the joint to activate and optimize acceleration for.
    */
   public void activateJoint(OneDoFJointBasics jointToActivate)
   {
      jointsToActivate.add(jointToActivate);
   }

   /**
    * Deactivates a joint such that it is assumed to be uncontrollable. However, its position and
    * velocity are still considered when optimizing for the other active joints.
    * <p>
    * If the joint was already deactivated, nothing happens.
    * </p>
    * 
    * @param jointToDeactivate the joint to deactivate.
    */
   public void deactivateJoint(OneDoFJointBasics jointToDeactivate)
   {
      jointsToDeactivate.add(jointToDeactivate);
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

   public boolean hasJointTorqueWeight()
   {
      return !Double.isNaN(jointTorqueWeight);
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
    * Whether the solver should compute the resulting joint torques from the solution.
    *
    * @return {@code true} if this command should result in the solver computing the joint torques.
    */
   public boolean hasComputeJointTorques()
   {
      return computeJointTorques != null;
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
    * Gets the value for {@code jointTorqueWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    *
    * @return the new value for {@code jointTorqueWeight}.
    * @see #hasJointTorqueWeight()
    * @see #setJointTorqueWeight(double)
    */
   public double getJointTorqueWeight()
   {
      return jointTorqueWeight;
   }

   /**
    * Gets the value for {@code jointVelocityLimitMode}.
    * <p>
    * It is equal to {@code null} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code jointVelocityLimitMode}.
    * @see #hashJointVelocityLimitMode()
    * @see #setJointVelocityLimitMode(ActivationState)
    */
   public ActivationState getJointVelocityLimitMode()
   {
      return jointVelocityLimitMode;
   }

   public ActivationState getComputeJointTorques()
   {
      return computeJointTorques;
   }

   /**
    * Gets the list of joints to activate in the optimization problem.
    * 
    * @return the joints to activate.
    */
   public List<OneDoFJointBasics> getJointsToActivate()
   {
      return jointsToActivate;
   }

   /**
    * Gets the list of joints to deactivate in the optimization problem.
    * 
    * @return the joints to deactivate.
    */
   public List<OneDoFJointBasics> getJointsToDeactivate()
   {
      return jointsToDeactivate;
   }

   @Override
   public void set(InverseKinematicsOptimizationSettingsCommand other)
   {
      commandId = other.commandId;
      jointVelocityWeight = other.jointVelocityWeight;
      jointAccelerationWeight = other.jointAccelerationWeight;
      jointTorqueWeight = other.jointTorqueWeight;
      jointVelocityLimitMode = other.jointVelocityLimitMode;
      computeJointTorques = other.computeJointTorques;
      jointsToActivate.clear();
      for (int i = 0; i < other.jointsToActivate.size(); i++)
         jointsToActivate.add(other.jointsToActivate.get(i));
      jointsToDeactivate.clear();
      for (int i = 0; i < other.jointsToDeactivate.size(); i++)
         jointsToDeactivate.add(other.jointsToDeactivate.get(i));
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
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

         if (commandId != other.commandId)
            return false;
         if (Double.compare(jointVelocityWeight, other.jointVelocityWeight) != 0)
            return false;
         if (Double.compare(jointAccelerationWeight, other.jointAccelerationWeight) != 0)
            return false;
         if (Double.compare(jointTorqueWeight, other.jointTorqueWeight) != 0)
            return false;
         if (jointVelocityLimitMode != other.jointVelocityLimitMode)
            return false;
         if (computeJointTorques != other.computeJointTorques)
            return false;
         if (!jointsToActivate.equals(other.jointsToActivate))
            return false;
         if (!jointsToDeactivate.equals(other.jointsToDeactivate))
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
