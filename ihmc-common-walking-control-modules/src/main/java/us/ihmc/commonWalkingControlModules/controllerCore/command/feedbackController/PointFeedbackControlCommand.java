package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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
 * {@link PointFeedbackControlCommand} is a command meant to be submit to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link PointFeedbackControlCommand} is to notify the feedback controller
 * dedicated to control the end-effector provided in {@link #set(RigidBodyBasics, RigidBodyBasics)}
 * that it is requested to run during the next control tick.
 * </p>
 * <p>
 * From control tick to control tick each feedback controller can be entirely configured or
 * reconfigured, and enabled (by submitting a command) or disabled (by NOT submitting a command).
 * </p>
 * <p>
 * All the data contained in this command is expressed in world to ensure that the feedback
 * controller can properly interpret it.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class PointFeedbackControlCommand implements FeedbackControlCommand<PointFeedbackControlCommand>
{
   private final FramePoint3D bodyFixedPointInEndEffectorFrame = new FramePoint3D();

   /** Represents the expected control mode to execute this command. */
   private WholeBodyControllerCoreMode controlMode = null;
   /** The desired position to use in the feedback controller. */
   private final FramePoint3D referencePositionInRootFrame = new FramePoint3D();
   /** The desired or (IK) feed-forward linear velocity to use in the feedback controller. */
   private final FrameVector3D referenceLinearVelocityInRootFrame = new FrameVector3D();
   /** The (ID) feed-forward linear acceleration to use in the feedback controller. */
   private final FrameVector3D referenceLinearAccelerationInRootFrame = new FrameVector3D();
   /** The (VMC) feed-forward force to use in the feedback controller. */
   private final FrameVector3D referenceForceInRootFrame = new FrameVector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final DefaultPID3DGains gains = new DefaultPID3DGains();
   /**
    * This is the reference frame in which the linear part of the gains are to be applied. If
    * {@code null}, it is applied in the control frame.
    */
   private ReferenceFrame linearGainsFrame = null;

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
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public PointFeedbackControlCommand()
   {
      spatialAccelerationCommand.setSelectionMatrixForLinearControl();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(PointFeedbackControlCommand other)
   {
      bodyFixedPointInEndEffectorFrame.setIncludingFrame(other.bodyFixedPointInEndEffectorFrame);
      controlMode = other.controlMode;
      referencePositionInRootFrame.setIncludingFrame(other.referencePositionInRootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(other.referenceLinearVelocityInRootFrame);
      referenceLinearAccelerationInRootFrame.setIncludingFrame(other.referenceLinearAccelerationInRootFrame);
      referenceForceInRootFrame.setIncludingFrame(other.referenceForceInRootFrame);

      gains.set(other.gains);
      linearGainsFrame = other.linearGainsFrame;
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
      resetBodyFixedPoint();
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
    * Resets the control base frame to its default value.
    *
    * @see #setControlBaseFrame(ReferenceFrame)
    */
   public void resetControlBaseFrame()
   {
      controlBaseFrame = null;
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
    * Sets the reference frame in which the gains should be applied.
    * <p>
    * If the reference frame is {@code null}, the gains will be applied in the control frame.
    * </p>
    *
    * @param linearGainsFrame the reference frame to use for the position gains.
    */
   public void setGainsFrame(ReferenceFrame linearGainsFrame)
   {
      this.linearGainsFrame = linearGainsFrame;
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
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse kinematics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code bodyFixedPoint} provided.
    * </p>
    *
    * @param desiredPosition the position the {@code bodyFixedPoint} should reach. Not modified.
    * @param feedForwardLinearVelocity the feed-forward linear velocity of the {@code bodyFixedPoint}
    *           with respect to the {@code base}. Not modified.
    */
   public void setInverseKinematics(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly feedForwardLinearVelocity)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      ReferenceFrame rootFrame = desiredPosition.getReferenceFrame().getRootFrame();
      referencePositionInRootFrame.setIncludingFrame(desiredPosition);
      referencePositionInRootFrame.changeFrame(rootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(feedForwardLinearVelocity);
      referenceLinearVelocityInRootFrame.changeFrame(rootFrame);
      referenceLinearAccelerationInRootFrame.setToZero(rootFrame);
      referenceForceInRootFrame.setToZero(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse dynamics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code bodyFixedPoint} provided.
    * </p>
    *
    * @param desiredPosition the position that the {@code bodyFixedPoint} should reach. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the {@code bodyFixedPoint} with
    *           respect to the {@code base}. Not modified.
    * @param feedForwardLinearAcceleration the feed-forward linear acceleration of the
    *           {@code bodyFixedPoint} with respect to the {@code base}. Not modified.
    */
   public void setInverseDynamics(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly desiredLinearVelocity,
                                  FrameVector3DReadOnly feedForwardLinearAcceleration)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      ReferenceFrame rootFrame = desiredPosition.getReferenceFrame().getRootFrame();
      referencePositionInRootFrame.setIncludingFrame(desiredPosition);
      referencePositionInRootFrame.changeFrame(rootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(desiredLinearVelocity);
      referenceLinearVelocityInRootFrame.changeFrame(rootFrame);
      referenceLinearAccelerationInRootFrame.setIncludingFrame(feedForwardLinearAcceleration);
      referenceLinearAccelerationInRootFrame.changeFrame(rootFrame);
      referenceForceInRootFrame.setToZero(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code bodyFixedPoint} provided.
    * </p>
    *
    * @param desiredPosition the position that the {@code bodyFixedPoint} should reach. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the {@code bodyFixedPoint} with
    *           respect to the {@code base}. Not modified.
    * @param feedForwardForce the feed-forward force to exert at {@code bodyFixedPoint}. Not modified.
    */
   public void setVirtualModelControl(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly desiredLinearVelocity, FrameVector3DReadOnly feedForwardForce)
   {
      setControlMode(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
      ReferenceFrame rootFrame = desiredPosition.getReferenceFrame().getRootFrame();
      referencePositionInRootFrame.setIncludingFrame(desiredPosition);
      referencePositionInRootFrame.changeFrame(rootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(desiredLinearVelocity);
      referenceLinearVelocityInRootFrame.changeFrame(rootFrame);
      referenceForceInRootFrame.setIncludingFrame(feedForwardForce);
      referenceForceInRootFrame.changeFrame(rootFrame);
      referenceLinearAccelerationInRootFrame.setToZero(rootFrame);
   }

   /**
    * Zeroes the offset of the {@code bodyFixedPoint} such that after calling this method <br>
    * {@code bodyFixedPoint == new FramePoint(endEffector.getBodyFixedFrame())}.
    */
   public void resetBodyFixedPoint()
   {
      bodyFixedPointInEndEffectorFrame.setToZero(getEndEffector().getBodyFixedFrame());
   }

   /**
    * Sets the position of the {@code bodyFixedPoint} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code bodyFixedPoint} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired position.
    * </p>
    *
    * @param bodyFixedPointInEndEffectorFrame the position of the {@code bodyFixedPoint}. Not modified.
    * @throws ReferenceFrameMismatchException if any the argument is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setBodyFixedPointToControl(FramePoint3DReadOnly bodyFixedPointInEndEffectorFrame)
   {
      bodyFixedPointInEndEffectorFrame.checkReferenceFrameMatch(getEndEffector().getBodyFixedFrame());
      this.bodyFixedPointInEndEffectorFrame.set(bodyFixedPointInEndEffectorFrame);
   }

   /**
    * This specifies that the 3 translational degrees of freedom of the end-effector are to be
    * controlled.
    */
   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixForLinearControl();
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
      spatialAccelerationCommand.setSelectionMatrixForLinearControl(selectionMatrix);
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
    * Sets the linear weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weightMatrix weight matrix holding the linear weights to use for each component of the
    *           desired acceleration. Not modified.
    */
   public void setWeightMatrix(WeightMatrix3D weightMatrix)
   {
      spatialAccelerationCommand.setLinearPartOfWeightMatrix(weightMatrix);
      spatialAccelerationCommand.setAngularWeightsToZero();
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weight the weight to use for each direction. Not modified.
    */
   public void setWeightsForSolver(Vector3DReadOnly weight)
   {
      spatialAccelerationCommand.setLinearWeights(weight);
      spatialAccelerationCommand.setAngularWeightsToZero();
   }

   public void getBodyFixedPointIncludingFrame(FramePoint3D bodyFixedPointToControlToPack)
   {
      bodyFixedPointToControlToPack.setIncludingFrame(bodyFixedPointInEndEffectorFrame);
   }

   public FramePoint3DBasics getBodyFixedPointToControl()
   {
      return bodyFixedPointInEndEffectorFrame;
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
    * Gets the reference position to use in the feedback controller.
    * <p>
    * The reference position typically represents the desired position.
    * </p>
    * 
    * @return the reference position.
    */
   public FramePoint3DBasics getReferencePosition()
   {
      return referencePositionInRootFrame;
   }

   /**
    * Gets the reference linear velocity to use in the feedback controller.
    * <p>
    * Depending on the active control mode, it can be used as a desired (ID & WMC) or a feed-forward
    * term (IK).
    * </p>
    * 
    * @return the reference linear velocity.
    */
   public FrameVector3DBasics getReferenceLinearVelocity()
   {
      return referenceLinearVelocityInRootFrame;
   }

   /**
    * Gets the reference linear acceleration to use in the feedback controller.
    * <p>
    * It is used in the inverse dynamics mode as a feed-forward term.
    * </p>
    * 
    * @return the reference linear acceleration.
    */
   public FrameVector3DBasics getReferenceLinearAcceleration()
   {
      return referenceLinearAccelerationInRootFrame;
   }

   /**
    * Gets the reference force to use in the feedback controller.
    * <p>
    * It is used in the virtual control mode as a feed-forward term.
    * </p>
    * 
    * @return the reference force.
    */
   public FrameVector3D getReferenceForce()
   {
      return referenceForceInRootFrame;
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

   public PID3DGains getGains()
   {
      return gains;
   }

   public ReferenceFrame getLinearGainsFrame()
   {
      return linearGainsFrame;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.POINT;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof PointFeedbackControlCommand)
      {
         PointFeedbackControlCommand other = (PointFeedbackControlCommand) object;

         if (controlMode != other.controlMode)
            return false;
         if (!bodyFixedPointInEndEffectorFrame.equals(other.bodyFixedPointInEndEffectorFrame))
            return false;
         if (!referencePositionInRootFrame.equals(other.referencePositionInRootFrame))
            return false;
         if (!referenceLinearVelocityInRootFrame.equals(other.referenceLinearVelocityInRootFrame))
            return false;
         if (!referenceLinearAccelerationInRootFrame.equals(other.referenceLinearAccelerationInRootFrame))
            return false;
         if (!referenceForceInRootFrame.equals(other.referenceForceInRootFrame))
            return false;
         if (!gains.equals(other.gains))
            return false;
         if (linearGainsFrame != other.linearGainsFrame)
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
      ret += "position = " + referencePositionInRootFrame;
      return ret;
   }
}
