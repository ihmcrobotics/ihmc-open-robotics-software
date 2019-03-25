package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;

/**
 * A {@code OneDoFJointFeedbackControlCommand} can be used to request the
 * {@link WholeBodyFeedbackController} to run PD controllers on a single {@link OneDoFJointBasics}
 * to reach given desired positions and desired velocities.
 * <p>
 * The PD controllers used also handle a feed-forward term for improved performance.
 * </p>
 * <p>
 * In addition to the desireds, the gains to be used in the PD controllers have to be provided
 * alongside with the weight used in the QP optimization problem, see for instance
 * {@link WholeBodyInverseDynamicsSolver}.
 * </p>
 * <p>
 * Every control tick, a {@code OneDoFJointFeedbackControlCommand} has to be sent to the controller
 * core allowing the higher-level controller to continuously update the desireds, gains, and weight
 * to use.
 * </p>
 *
 *
 * @author Sylvain Bertrand
 */
public class OneDoFJointFeedbackControlCommand implements FeedbackControlCommand<OneDoFJointFeedbackControlCommand>
{
   private OneDoFJointBasics joint;
   /** Represents the expected control mode to execute this command. */
   private WholeBodyControllerCoreMode controlMode = null;
   /** The desired joint position. */
   private double referencePosition = 0.0;
   /** The desired or (IK) feed-forward joint velocity. */
   private double referenceVelocity = 0.0;
   /** The (ID) feed-forward joint acceleration. */
   private double referenceAcceleration = 0.0;
   /** The (VMC) feed-forward joint effort (force or torque depending on the type of joint). */
   private double referenceEffort = 0.0;
   /** Weight used in the QP optimization describing how 'important' achieving this command is. */
   private double weightForSolver = Double.POSITIVE_INFINITY;
   /** Gains to used by the PD controllers for the next control tick. */
   private final PDGains gains = new PDGains();

   /**
    * Creates an empty command.
    */
   public OneDoFJointFeedbackControlCommand()
   {
   }

   public void clear()
   {
      controlMode = null;
      joint = null;
      referencePosition = 0.0;
      referenceVelocity = 0.0;
      referenceAcceleration = 0.0;
      referenceEffort = 0.0;
      weightForSolver = Double.POSITIVE_INFINITY;
      gains.set(0.0, 0.0, 0.0, 0.0);
   }

   @Override
   public void set(OneDoFJointFeedbackControlCommand other)
   {
      controlMode = other.controlMode;
      joint = other.joint;
      referencePosition = other.referencePosition;
      referenceVelocity = other.referenceVelocity;
      referenceAcceleration = other.referenceAcceleration;
      referenceEffort = other.referenceEffort;
      weightForSolver = other.weightForSolver;
      gains.set(other.gains);
   }

   /**
    * Sets the joint that is to be controller with this command.
    * 
    * @param joint the joint to control.
    */
   public void setJoint(OneDoFJointBasics joint)
   {
      this.joint = joint;
   }

   /**
    * Sets the expected control mode that the controller core should be using to execute this command.
    * <p>
    * Note that the control mode is updated when calling either the main input setters, i.e.
    * {@code this.setInverseKinematics(...)}, {@code this.setInverseDynamics(...)}, or
    * {@code this.setVirtualControlModel(...)}.
    * </p>
    * <p>
    * This is a safety feature, the controller core will throw an exception in the case the control
    * mode mismatches the active mode of the controller core.
    * </p>
    * 
    * @param controlMode the expected control mode.
    */
   public void setControlMode(WholeBodyControllerCoreMode controlMode)
   {
      this.controlMode = controlMode;
   }

   /**
    * Updates the gains to be used for the next control tick.
    *
    * @param gains the new set of gains to be used. The data is copied into a local object. Not
    *           modified.
    */
   public void setGains(PDGainsReadOnly gains)
   {
      this.gains.set(gains);
   }

   /**
    * Updates the weight to be used for the next control tick.
    * <p>
    * This relates to how 'important' is this command compared to other commands submitted.
    * </p>
    *
    * @param weight the new joint weight for the QP optimization.
    */
   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
   }

   /**
    * Configures this feedback command's inputs for inverse kinematics.
    * 
    * @param desiredPosition the desired position the joint should reach.
    * @param feedForwardVelocity the feed-forward joint velocity.
    */
   public void setInverseKinematics(double desiredPosition, double feedForwardVelocity)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      referencePosition = desiredPosition;
      referenceVelocity = feedForwardVelocity;
      referenceAcceleration = 0.0;
      referenceEffort = 0.0;
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * 
    * @param desiredPosition the desired position the joint should reach.
    * @param desiredVelocity the desired velocity the joint should reach.
    * @param feedForwardAcceleration the feed-forward joint acceleration.
    */
   public void setInverseDynamics(double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      referencePosition = desiredPosition;
      referenceVelocity = desiredVelocity;
      referenceAcceleration = feedForwardAcceleration;
      referenceEffort = 0.0;
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * 
    * @param desiredPosition the desired position the joint should reach.
    * @param desiredVelocity the desired velocity the joint should reach.
    * @param feedForwardEffort the feed-forward joint effort, i.e. joint torque or force depending on
    *           its type.
    */
   public void setVirtualModelControl(double desiredPosition, double desiredVelocity, double feedForwardEffort)
   {
      setControlMode(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
      referencePosition = desiredPosition;
      referenceVelocity = desiredVelocity;
      referenceAcceleration = 0.0;
      referenceEffort = feedForwardEffort;
   }

   /**
    * Sets the reference joint position.
    * <p>
    * Always prefer using either {@code this.setInverseKinematics(...)},
    * {@code this.setInverseDynamics(...)}, or {@code this.setVirtualControlModel(...)}.
    * </p>
    * 
    * @param referencePosition the reference joint position.
    */
   public void setReferencePosition(double referencePosition)
   {
      this.referencePosition = referencePosition;
   }

   /**
    * Sets the reference joint velocity.
    * <p>
    * Always prefer using either {@code this.setInverseKinematics(...)},
    * {@code this.setInverseDynamics(...)}, or {@code this.setVirtualControlModel(...)}.
    * </p>
    * 
    * @param referenceVelocity the reference joint velocity.
    */
   public void setReferenceVelocity(double referenceVelocity)
   {
      this.referenceVelocity = referenceVelocity;
   }

   /**
    * Sets the reference joint acceleration.
    * <p>
    * Always prefer using either {@code this.setInverseKinematics(...)},
    * {@code this.setInverseDynamics(...)}, or {@code this.setVirtualControlModel(...)}.
    * </p>
    * 
    * @param referenceAcceleration the reference joint acceleration.
    */
   public void setReferenceAcceleration(double referenceAcceleration)
   {
      this.referenceAcceleration = referenceAcceleration;
   }

   /**
    * Sets the reference joint effort, i.e. torque or force depending on the joint type.
    * <p>
    * Always prefer using either {@code this.setInverseKinematics(...)},
    * {@code this.setInverseDynamics(...)}, or {@code this.setVirtualControlModel(...)}.
    * </p>
    * 
    * @param referenceEffort the reference joint effort.
    */
   public void setReferenceEffort(double referenceEffort)
   {
      this.referenceEffort = referenceEffort;
   }

   /**
    * Gets the expected control mode to execute this command with.
    * 
    * @return the expected active controller core control mode.
    */
   public WholeBodyControllerCoreMode getControlMode()
   {
      return controlMode;
   }

   /**
    * The joint control with this command.
    * 
    * @return the joint.
    */
   public OneDoFJointBasics getJoint()
   {
      return joint;
   }

   /**
    * Gets the reference joint position to use in the feedback controller.
    * <p>
    * The reference position typically represents the desired position.
    * </p>
    * 
    * @return the reference position.
    */
   public double getReferencePosition()
   {
      return referencePosition;
   }

   /**
    * Gets the reference joint velocity to use in the feedback controller.
    * <p>
    * Depending on the active control mode, it can be used as a desired (ID & WMC) or a feed-forward
    * term (IK).
    * </p>
    * 
    * @return the reference joint velocity.
    */
   public double getReferenceVelocity()
   {
      return referenceVelocity;
   }

   /**
    * Gets the reference joint acceleration to use in the feedback controller.
    * <p>
    * It is used in the inverse dynamics mode as a feed-forward term.
    * </p>
    * 
    * @return the reference joint acceleration.
    */
   public double getReferenceAcceleration()
   {
      return referenceAcceleration;
   }

   /**
    * Gets the reference joint effort to use in the feedback controller.
    * <p>
    * It is used in the virtual control mode as a feed-forward term.
    * </p>
    * 
    * @return the reference joint effort.
    */
   public double getReferenceEffort()
   {
      return referenceEffort;
   }

   /**
    * Gets the weight to use with this command in the QP optimization.
    * 
    * @return the solver's weight for this command.
    */
   public double getWeightForSolver()
   {
      return weightForSolver;
   }

   /**
    * Gets the gains to use in the feedback controller with this commands.
    * 
    * @return the gains.
    */
   public PDGains getGains()
   {
      return gains;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINTSPACE;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof OneDoFJointFeedbackControlCommand)
      {
         OneDoFJointFeedbackControlCommand other = (OneDoFJointFeedbackControlCommand) object;

         if (controlMode != other.controlMode)
            return false;
         if (joint != other.joint)
            return false;
         if (Double.compare(referencePosition, other.referencePosition) != 0)
            return false;
         if (Double.compare(referenceVelocity, other.referenceVelocity) != 0)
            return false;
         if (Double.compare(referenceAcceleration, other.referenceAcceleration) != 0)
            return false;
         if (Double.compare(referenceEffort, other.referenceEffort) != 0)
            return false;
         if (Double.compare(weightForSolver, other.weightForSolver) != 0)
            return false;
         if (!gains.equals(other.gains))
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
      return getClass().getSimpleName() + ": control mode: " + controlMode + ", q: " + referencePosition + ", qd: " + referenceVelocity + ", qdd: "
            + referenceAcceleration + ", tau: " + referenceEffort;
   }
}
