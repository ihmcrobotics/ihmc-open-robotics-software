package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

/**
 * A {@code OrientationFeedbackControlCommand} can be used to request the
 * {@link WholeBodyFeedbackController} to run a PD controller in that task space of an end-effector
 * given its desired orientation and angular velocity.
 * <p>
 * The PD controller used also handle a feed-forward angular acceleration for improved tracking
 * performance.
 * </p>
 * <p>
 * In addition to the desireds, the gains to be used in the PD controller have to be provided
 * alongside with the weight used in the QP optimization problem, see for instance
 * {@link WholeBodyInverseDynamicsSolver}.
 * </p>
 * Every control tick, a {@code OrientationFeedbackControlCommand} has to be sent to the controller
 * core allowing the higher-level controller to continuously update the desireds, gains, and weight
 * to use.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class OrientationFeedbackControlCommand implements FeedbackControlCommand<OrientationFeedbackControlCommand>
{
   // TODO: This is not used by the controller core. The control point orientation is only used with the spatial control command.
   private final FrameQuaternion bodyFixedOrientationInEndEffectorFrame = new FrameQuaternion();

   /** Represents the expected control mode to execute this command. */
   private WholeBodyControllerCoreMode controlMode = null;
   /** The desired orientation to use in the feedback controller. */
   private final FrameQuaternion referenceOrientation = new FrameQuaternion();
   /** The desired or (IK) feed-forward angular velocity to use in the feedback controller. */
   private final FrameVector3D referenceAngularVelocity = new FrameVector3D();
   /** The (ID) feed-forward angular acceleration to use in the feedback controller. */
   private final FrameVector3D referenceAngularAcceleration = new FrameVector3D();
   /** The (VMC) feed-forward torque to use in the feedback controller. */
   private final FrameVector3D referenceTorque = new FrameVector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final DefaultPID3DGains gains = new DefaultPID3DGains();
   /**
    * This is the reference frame in which the angular part of the gains are to be applied. If
    * {@code null}, it is applied in the control frame.
    */
   private ReferenceFrame angularGainsFrame = null;

   /**
    * Acceleration command used to save different control properties such as: the end-effector, the
    * base, and the weight to be used in the QP optimization.
    * <p>
    * Should not be accessed from the user side.
    * </p>
    */
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   /**
    * The control base frame is the reference frame with respect to which the end-effector is to be
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect to
    * the control base frame.
    */
   private ReferenceFrame controlBaseFrame = null;

   /**
    * Creates an empty command.
    */
   public OrientationFeedbackControlCommand()
   {
      spatialAccelerationCommand.setSelectionMatrixForAngularControl();
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not modified.
    */
   @Override
   public void set(OrientationFeedbackControlCommand other)
   {
      bodyFixedOrientationInEndEffectorFrame.setIncludingFrame(other.bodyFixedOrientationInEndEffectorFrame);
      controlMode = other.controlMode;
      referenceOrientation.setIncludingFrame(other.referenceOrientation);
      referenceAngularVelocity.setIncludingFrame(other.referenceAngularVelocity);
      referenceAngularAcceleration.setIncludingFrame(other.referenceAngularAcceleration);
      referenceTorque.setIncludingFrame(other.referenceTorque);

      gains.set(other.gains);
      angularGainsFrame = other.angularGainsFrame;
      spatialAccelerationCommand.set(other.spatialAccelerationCommand);
      controlBaseFrame = other.controlBaseFrame;
   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be to control the end-effector.
    * </p>
    *
    * @param base the rigid-body located right before the first joint to be used for controlling the
    *           end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBodyBasics base, RigidBodyBasics endEffector)
   {
      spatialAccelerationCommand.set(base, endEffector);
      resetBodyFixedOrientation();
   }

   /**
    * Intermediate base located between the {@code base} and {@code endEffector}.
    * <p>
    * This parameter is optional. If provided, it is used to improve singularity avoidance by applying
    * a privileged joint configuration to the kinematic chain going from {@code primaryBase} to
    * {@code endEffector}.
    * </p>
    * <p>
    * Here is an example of application: {@code endEffector == leftHand},
    * {@code base == rootJoint.getPredecessor()} such that to control the {@code leftHand}, the
    * controller core uses the arm joints, the spine joints, and also the non-actuated floating joint.
    * If {@code primaryBase == chest}, as soon as the left arm comes close to a singular configuration
    * such as a straight elbow, the privileged configuration framework will help bending the elbow.
    * This reduces the time needed to escape the singular configuration. It also prevents unfortunate
    * situation where the elbow would try to bend past the joint limit.
    * </p>
    *
    * @param primaryBase
    */
   public void setPrimaryBase(RigidBodyBasics primaryBase)
   {
      spatialAccelerationCommand.setPrimaryBase(primaryBase);
   }

   /**
    * The control base frame is the reference frame with respect to which the end-effector is to be
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect to
    * the control base frame.
    *
    * @param controlBaseFrame the new control base frame.
    */
   public void setControlBaseFrame(ReferenceFrame controlBaseFrame)
   {
      if (controlBaseFrame == getBase().getBodyFixedFrame())
         this.controlBaseFrame = null;
      else if (controlBaseFrame.isAStationaryFrame() || controlBaseFrame instanceof MovingReferenceFrame)
         this.controlBaseFrame = controlBaseFrame;
      else
         throw new IllegalArgumentException("The control base frame has to either be a stationary frame or a MovingReferenceFrame.");
   }

   /**
    * Sets the gains to use during the next control tick.
    *
    * @param gains the new set of gains to use. Not modified.
    */
   public void setGains(PID3DGainsReadOnly gains)
   {
      this.gains.set(gains);
   }

   /**
    * Resets the control base frame to its default value.
    *
    * @see #setControlBaseFrame(ReferenceFrame)
    */
   public void resetControlBaseFrame()
   {
      controlBaseFrame = null;
   }

   /**
    * Sets the reference frame in which the gains should be applied.
    * <p>
    * If the reference frame is {@code null}, the gains will be applied in the control frame.
    * </p>
    *
    * @param angularGainsFrame the reference frame to use for the orientation gains.
    */
   public void setGainsFrame(ReferenceFrame angularGainsFrame)
   {
      this.angularGainsFrame = angularGainsFrame;
   }

   /**
    * Sets the expected control mode that the controller core should be using to execute this command.
    * <p>
    * Note that the control mode is updated when calling either the main input setters, i.e.
    * {@code this.setInverseKinematics(...)}, {@code this.setInverseDynamics(...)}, or
    * {@code this.setVirtualControlMode(...)}.
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
    * Configures this feedback command's inputs for inverse kinematics.
    * <p>
    * Sets the desired data expressed in trajectory frame to be used during the next control tick and sets
    * the control mode for inverse kinematics.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code endEffector.getBodyFixedFrame()} should
    *           reach. Not modified.
    * @param feedForwardAngularVelocity the feed-forward angular velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not modified.
    */
   public void setInverseKinematics(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly feedForwardAngularVelocity)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      ReferenceFrame trajectoryFrame = desiredOrientation.getReferenceFrame();
      referenceOrientation.setIncludingFrame(desiredOrientation);
      referenceAngularVelocity.setIncludingFrame(feedForwardAngularVelocity);
      referenceAngularVelocity.checkReferenceFrameMatch(trajectoryFrame);
      referenceAngularAcceleration.setToZero(trajectoryFrame);
      referenceTorque.setToZero(trajectoryFrame);
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in trajectory frame to be used during the next control tick and sets
    * the control mode for inverse dynamics.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code endEffector.getBodyFixedFrame()} should
    *           reach. Not modified.
    * @param desiredAngularVelocity the desired angular velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not modified.
    * @param feedForwardAngularAcceleration the feed-forward angular acceleration of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not modified.
    */
   public void setInverseDynamics(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly desiredAngularVelocity,
                                  FrameVector3DReadOnly feedForwardAngularAcceleration)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      ReferenceFrame trajectoryFrame = desiredOrientation.getReferenceFrame();
      referenceOrientation.setIncludingFrame(desiredOrientation);
      referenceAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      referenceAngularVelocity.checkReferenceFrameMatch(trajectoryFrame);
      referenceAngularAcceleration.setIncludingFrame(feedForwardAngularAcceleration);
      referenceAngularAcceleration.checkReferenceFrameMatch(trajectoryFrame);
      referenceTorque.setToZero(trajectoryFrame);
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in trajectory frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code endEffector.getBodyFixedFrame()} should
    *           reach. Not modified.
    * @param desiredAngularVelocity the desired angular velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not modified.
    * @param feedForwardTorque the feed-forward torque to exert at
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setVirtualModelControl(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly desiredAngularVelocity,
                                      FrameVector3DReadOnly feedForwardTorque)
   {
      setControlMode(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
      ReferenceFrame trajectoryFrame = desiredOrientation.getReferenceFrame();
      referenceOrientation.setIncludingFrame(desiredOrientation);
      referenceAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      referenceAngularVelocity.checkReferenceFrameMatch(trajectoryFrame);
      referenceTorque.setIncludingFrame(feedForwardTorque);
      referenceTorque.checkReferenceFrameMatch(trajectoryFrame);
      referenceAngularAcceleration.setToZero(trajectoryFrame);
   }

   /**
    * Zeroes the offset of the {@code bodyFixedOrientation} such that after calling this method <br>
    * {@code bodyFixedOrientation == new FrameQuaternion(endEffector.getBodyFixedFrame())}.
    */
   public void resetBodyFixedOrientation()
   {
      bodyFixedOrientationInEndEffectorFrame.setToZero(getEndEffector().getBodyFixedFrame());
   }

   /**
    * Sets the position of the {@code bodyFixedOrientation} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code bodyFixedOrientation} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired orientation.
    * </p>
    *
    * @param bodyFixedOrientationInEndEffectorFrame the position of the {@code bodyFixedOrientation}.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if any the argument is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setBodyFixedOrientationToControl(FrameQuaternionReadOnly bodyFixedOrientationInEndEffectorFrame)
   {
      bodyFixedOrientationInEndEffectorFrame.checkReferenceFrameMatch(getEndEffector().getBodyFixedFrame());
      this.bodyFixedOrientationInEndEffectorFrame.set(bodyFixedOrientationInEndEffectorFrame);
   }

   /**
    * This specifies that the 3 rotational degrees of freedom of the end-effector are to be controlled.
    */
   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixForAngularControl();
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector that
    * are to be controlled. It is initialized such that the controller will by default control all the
    * end-effector DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    *
    * @param selectionMatrix the selection matrix to copy data from. Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      spatialAccelerationCommand.setSelectionMatrixForAngularControl(selectionMatrix);
   }

   /**
    * Sets the weight to use in the optimization problem.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weight the weight value to use for this command.
    */
   public void setWeightForSolver(double weight)
   {
      spatialAccelerationCommand.setWeight(weight);
   }

   /**
    * Sets the angular weights to use in the optimization problem for each individual degree of
    * freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angularWeightMatrix weight matrix holding the angular weights to use for each component of
    *           the desired acceleration. Not modified.
    */
   public void setWeightMatrix(WeightMatrix3D angularWeightMatrix)
   {
      spatialAccelerationCommand.setAngularPartOfWeightMatrix(angularWeightMatrix);
      spatialAccelerationCommand.setLinearWeightsToZero();
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angular the weights to use for the angular part of this command. Not modified.
    */
   public void setWeightsForSolver(Vector3DReadOnly angular)
   {
      spatialAccelerationCommand.setAngularWeights(angular);
      spatialAccelerationCommand.setLinearWeightsToZero();
   }

   public void getBodyFixedOrientationIncludingFrame(FrameQuaternion bodyFixedOrientationToControlToPack)
   {
      bodyFixedOrientationToControlToPack.setIncludingFrame(bodyFixedOrientationInEndEffectorFrame);
   }

   public FrameQuaternionBasics getBodyFixedOrientationToControl()
   {
      return bodyFixedOrientationInEndEffectorFrame;
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
    * Gets the reference orientation to use in the feedback controller.
    * <p>
    * The reference orientation typically represents the desired orientation.
    * </p>
    * 
    * @return the reference orientation.
    */
   public FrameQuaternionBasics getReferenceOrientation()
   {
      return referenceOrientation;
   }

   /**
    * Gets the reference angular velocity to use in the feedback controller.
    * <p>
    * Depending on the active control mode, it can be used as a desired (ID & WMC) or a feed-forward
    * term (IK).
    * </p>
    * 
    * @return the reference angular velocity.
    */
   public FrameVector3DBasics getReferenceAngularVelocity()
   {
      return referenceAngularVelocity;
   }

   /**
    * Gets the reference angular acceleration to use in the feedback controller.
    * <p>
    * It is used in the inverse dynamics mode as a feed-forward term.
    * </p>
    * 
    * @return the reference angular acceleration.
    */
   public FrameVector3DBasics getReferenceAngularAcceleration()
   {
      return referenceAngularAcceleration;
   }

   /**
    * Gets the reference torque to use in the feedback controller.
    * <p>
    * It is used in the virtual control mode as a feed-forward term.
    * </p>
    * 
    * @return the reference torque.
    */
   public FrameVector3DBasics getReferenceTorque()
   {
      return referenceTorque;
   }

   public RigidBodyBasics getBase()
   {
      return spatialAccelerationCommand.getBase();
   }

   public RigidBodyBasics getEndEffector()
   {
      return spatialAccelerationCommand.getEndEffector();
   }

   public ReferenceFrame getControlBaseFrame()
   {
      if (controlBaseFrame != null)
         return controlBaseFrame;
      else
         return spatialAccelerationCommand.getBase().getBodyFixedFrame();
   }

   public SpatialAccelerationCommand getSpatialAccelerationCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.ORIENTATION;
   }

   public PID3DGains getGains()
   {
      return gains;
   }

   public ReferenceFrame getAngularGainsFrame()
   {
      return angularGainsFrame;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof OrientationFeedbackControlCommand)
      {
         OrientationFeedbackControlCommand other = (OrientationFeedbackControlCommand) object;

         if (controlMode != other.controlMode)
            return false;
         if (!bodyFixedOrientationInEndEffectorFrame.equals(other.bodyFixedOrientationInEndEffectorFrame))
            return false;
         if (!referenceOrientation.equals(other.referenceOrientation))
            return false;
         if (!referenceAngularVelocity.equals(other.referenceAngularVelocity))
            return false;
         if (!referenceAngularAcceleration.equals(other.referenceAngularAcceleration))
            return false;
         if (!referenceTorque.equals(other.referenceTorque))
            return false;
         if (!gains.equals(other.gains))
            return false;
         if (angularGainsFrame != other.angularGainsFrame)
            return false;
         if (!spatialAccelerationCommand.equals(other.spatialAccelerationCommand))
            return false;
         if (controlBaseFrame != other.controlBaseFrame)
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
      String ret = getClass().getSimpleName() + ": ";
      ret += "base = " + spatialAccelerationCommand.getBase() + ", ";
      ret += "endEffector = " + spatialAccelerationCommand.getEndEffector() + ", ";
      ret += "orientation = " + referenceOrientation.toStringAsYawPitchRoll();
      return ret;
   }
}
