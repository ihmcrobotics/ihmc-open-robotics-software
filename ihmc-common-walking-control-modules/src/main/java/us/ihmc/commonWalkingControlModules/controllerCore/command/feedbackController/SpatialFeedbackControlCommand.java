package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link SpatialFeedbackControlCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link SpatialFeedbackControlCommand} is to notify the feedback controller
 * dedicated to control the end-effector provided in {@link #set(RigidBody, RigidBody)} that it is
 * requested to run during the next control tick.
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
public class SpatialFeedbackControlCommand implements FeedbackControlCommand<SpatialFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3D controlFrameOriginInEndEffectorFrame = new Point3D();
   private final Quaternion controlFrameOrientationInEndEffectorFrame = new Quaternion();

   private final Point3D desiredPositionInWorld = new Point3D();
   private final Vector3D desiredLinearVelocityInWorld = new Vector3D();
   private final Vector3D feedForwardLinearAccelerationInWorld = new Vector3D();

   private final Quaternion desiredOrientationInWorld = new Quaternion();
   private final Vector3D desiredAngularVelocityInWorld = new Vector3D();
   private final Vector3D feedForwardAngularAccelerationInWorld = new Vector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
   /** This is the reference frame in which the angular part of the gains are to be applied. If {@code null}, it is applied in the control frame. */
   private ReferenceFrame angularGainsFrame = null;
   /** This is the reference frame in which the linear part of the gains are to be applied. If {@code null}, it is applied in the control frame. */
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
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect
    * to the control base frame.
    */
   private ReferenceFrame controlBaseFrame = null;

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public SpatialFeedbackControlCommand()
   {
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(SpatialFeedbackControlCommand other)
   {
      spatialAccelerationCommand.set(other.spatialAccelerationCommand);

      controlFrameOriginInEndEffectorFrame.set(other.controlFrameOriginInEndEffectorFrame);
      controlFrameOrientationInEndEffectorFrame.set(other.controlFrameOrientationInEndEffectorFrame);

      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);

      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      desiredAngularVelocityInWorld.set(other.desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationInWorld.set(other.feedForwardAngularAccelerationInWorld);

      controlBaseFrame = other.controlBaseFrame;

      gains.set(other.gains);
   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be used to control the end-effector.
    * </p>
    *
    * @param base the rigid-body located right before the first joint to be used for controlling the
    *           end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBody base, RigidBody endEffector)
   {
      spatialAccelerationCommand.set(base, endEffector);
   }

   /**
    * Intermediate base located between the {@code base} and {@code endEffector}.
    * <p>
    * This parameter is optional. If provided, it is used to improve singularity avoidance by
    * applying a privileged joint configuration to the kinematic chain going from
    * {@code primaryBase} to {@code endEffector}.
    * </p>
    * <p>
    * Here is an example of application: {@code endEffector == leftHand},
    * {@code base == rootJoint.getPredecessor()} such that to control the {@code leftHand}, the
    * controller core uses the arm joints, the spine joints, and also the non-actuated floating
    * joint. If {@code primaryBase == chest}, as soon as the left arm comes close to a singular
    * configuration such as a straight elbow, the privileged configuration framework will help
    * bending the elbow. This reduces the time needed to escape the singular configuration. It also
    * prevents unfortunate situation where the elbow would try to bend past the joint limit.
    * </p>
    *
    * @param primaryBase
    */
   public void setPrimaryBase(RigidBody primaryBase)
   {
      spatialAccelerationCommand.setPrimaryBase(primaryBase);
   }

   /**
    * The control base frame is the reference frame with respect to which the end-effector is to be
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect
    * to the control base frame.
    *
    * @param controlBaseFrame the new control base frame.
    */
   public void setControlBaseFrame(ReferenceFrame controlBaseFrame)
   {
      if (controlBaseFrame.isAStationaryFrame() || controlBaseFrame instanceof MovingReferenceFrame)
         this.controlBaseFrame = controlBaseFrame;
      else
         throw new IllegalArgumentException("The control base frame has to either be a stationary frame or a MovingReferenceFrame.");
   }

   /**
    * Sets whether or not to scale the weights on the joints below the intermediate base defined by
    * {@link #setPrimaryBase(RigidBody)}. Indicates that we would like to custom scale the weights
    * on the joints in the kinematic chain below the {@code primaryBase} when controlling the
    * {@code endEffector}.
    * <p>
    * If false, as is the case in the default setting, the controller uses the default scaling
    * factor
    * {@link us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator#secondaryTaskJointsWeight}.
    * </p>
    * <p>
    * If true, the controller uses the custom defined scaling factor
    * {@param secondaryTaskJointWeightScale} to scale the weights before the {@code primaryBase} in
    * the kinematic chain between the {@code base} and {@code endEffector}.
    * </p>
    * <p>
    * A scale factor greater than 1.0 indicates that it is desired to use the joints in the
    * kinematic chain between {@code base} and {@code primaryBase} to control the
    * {@code endEffector} more than the joints between the {@code primaryBase} and the
    * {@code endEffector}. For example, this can be used to say that we would prefer to use the
    * pelvis to control the foot acceleration than the leg joints.
    * </p>
    * <p>
    * A scale factor less than 1.0 indicates that it is desired to use the joints in the kinematic
    * chain between {@code primaryBase} and {@code endEffector} to control the {@code endEffector}
    * more than the joints between the {@code base} and the {@code primaryBase}. For example, this
    * can be used to say that we would prefer to use the leg joints to control the foot acceleration
    * than the pelvis.
    * </p>
    *
    * @param scaleSecondaryTaskJointWeight whether or not to use a custom scaling factor on the
    *           joints below the primary base. Optional.
    * @param secondaryTaskJointWeightScale custom scaling factor for the joints below the primary
    *           base. Optional.
    */
   public void setScaleSecondaryTaskJointWeight(boolean scaleSecondaryTaskJointWeight, double secondaryTaskJointWeightScale)
   {
      spatialAccelerationCommand.setScaleSecondaryTaskJointWeight(scaleSecondaryTaskJointWeight, secondaryTaskJointWeightScale);
   }

   /**
    * Resets the secondary task joint weight scaling factor on the joints below the
    * {@code primaryBase} to its default value.
    */
   public void resetSecondaryTaskJointWeightScale()
   {
      spatialAccelerationCommand.resetSecondaryTaskJointWeightScale();
   }

   /**
    * Sets the gains for both the position and orientation to use during the next control tick.
    *
    * @param gains the new set of gains to use. Not modified.
    */
   public void setGains(PIDSE3GainsReadOnly gains)
   {
      this.gains.set(gains);
   }

   /**
    * Sets only the orientation gains to use during the next control tick.
    *
    * @param orientationGains the new set of orientation gains to use. Not modified.
    */
   public void setOrientationGains(PID3DGainsReadOnly orientationGains)
   {
      this.gains.setOrientationGains(orientationGains);
   }

   /**
    * Sets only the position gains to use during the next control tick.
    *
    * @param positionGains the new set of position gains to use. Not modified.
    */
   public void setPositionGains(PID3DGainsReadOnly positionGains)
   {
      this.gains.setPositionGains(positionGains);
   }

   /**
    * Sets the reference frames in which the gains should be applied.
    * <p>
    * If a reference frame is {@code null}, the corresponding gains will be applied in the control frame.
    * </p>
    *
    * @param angularGainsFrame the reference frame to use for the orientation gains.
    * @param linearGainsFrame the reference frame to use for the position gains.
    */
   public void setGainsFrames(ReferenceFrame angularGainsFrame, ReferenceFrame linearGainsFrame)
   {
      this.angularGainsFrame = angularGainsFrame;
      this.linearGainsFrame = linearGainsFrame;
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    * <p>
    * The desired linear/angular velocity and feed-forward linear/angular acceleration are set to
    * zero.
    * </p>
    *
    * @param desiredPose describes the pose that the {@code controlFrame} should reach. It does NOT
    *           describe the desired pose of {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePose desiredPose)
   {
      desiredPose.checkReferenceFrameMatch(worldFrame);

      desiredPose.getPosition(desiredPositionInWorld);
      desiredPose.getOrientation(desiredOrientationInWorld);
      desiredLinearVelocityInWorld.setToZero();
      desiredAngularVelocityInWorld.setToZero();
      feedForwardLinearAccelerationInWorld.setToZero();
      feedForwardAngularAccelerationInWorld.setToZero();
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    * <p>
    * The desired linear velocity and feed-forward linear acceleration are set to zero.
    * </p>
    *
    * @param desiredPosition describes the position that the {@code controlFrame} should reach. It
    *           does NOT describe the desired position of {@code endEffector.getBodyFixedFrame()}.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePoint3D desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);

      desiredPositionInWorld.set(desiredPosition);
      desiredLinearVelocityInWorld.setToZero();
      feedForwardLinearAccelerationInWorld.setToZero();
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredPosition describes the position that the {@code controlFrame} should reach. It
    *           does NOT describe the desired position of {@code endEffector.getBodyFixedFrame()}.
    *           Not modified.
    * @param desiredLinearVelocity describes the desired linear velocity of the
    *           {@code controlFrame}'s origin with respect to the {@code base}. It does NOT describe
    *           the desired linear velocity of {@code endEffector.getBodyFixedFrame()}'s origin. Not
    *           modified.
    * @param feedForwardLinearAcceleration describes the desired linear acceleration of the
    *           {@code controlFrame}'s origin with respect to the {@code base}. It does NOT describe
    *           the desired linear acceleration of {@code endEffector.getBodyFixedFrame()}'s origin.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if any of the three arguments is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePoint3D desiredPosition, FrameVector3D desiredLinearVelocity, FrameVector3D feedForwardLinearAcceleration)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);
      desiredLinearVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardLinearAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredPositionInWorld.set(desiredPosition);
      desiredLinearVelocityInWorld.set(desiredLinearVelocity);
      feedForwardLinearAccelerationInWorld.set(feedForwardLinearAcceleration);
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    * <p>
    * The desired angular velocity and feed-forward angular acceleration are set to zero.
    * </p>
    *
    * @param desiredOrientation describes the orientation that the {@code controlFrame} should
    *           reach. It does NOT describe the desired orientation of
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FrameQuaternion desiredOrientation)
   {
      desiredOrientation.checkReferenceFrameMatch(worldFrame);

      desiredOrientationInWorld.set(desiredOrientation);
      desiredAngularVelocityInWorld.setToZero();
      feedForwardAngularAccelerationInWorld.setToZero();
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredOrientation describes the orientation that the {@code controlFrame} should
    *           reach. It does NOT describe the desired orientation of
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param desiredAngularVelocity describes the desired linear velocity of {@code controlFrame}
    *           with respect to the {@code base}. It is equivalent to the desired angular velocity
    *           of {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param feedForwardAngularAcceleration describes the desired linear acceleration of
    *           {@code controlFrame} with respect to the {@code base}. It is equivalent to the
    *           desired angular acceleration of {@code endEffector.getBodyFixedFrame()}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if any of the three arguments is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FrameQuaternion desiredOrientation, FrameVector3D desiredAngularVelocity, FrameVector3D feedForwardAngularAcceleration)
   {
      desiredOrientation.checkReferenceFrameMatch(worldFrame);
      desiredAngularVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardAngularAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredOrientationInWorld.set(desiredOrientation);
      desiredAngularVelocityInWorld.set(desiredAngularVelocity);
      feedForwardAngularAccelerationInWorld.set(feedForwardAngularAcceleration);
   }

   /**
    * Change the reference frame of the given data such that it is expressed in
    * {@link ReferenceFrame#getWorldFrame()}. The data will be used for the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredPosition describes the position that the {@code controlFrame} should reach. It
    *           does NOT describe the desired position of {@code endEffector.getBodyFixedFrame()}.
    *           Modified.
    * @param desiredLinearVelocity describes the desired linear velocity of the
    *           {@code controlFrame}'s origin with respect to the {@code base}. It does NOT describe
    *           the desired linear velocity of {@code endEffector.getBodyFixedFrame()}'s origin.
    *           Modified.
    * @param feedForwardLinearAcceleration describes the desired linear acceleration of the
    *           {@code controlFrame}'s origin with respect to the {@code base}. It does NOT describe
    *           the desired linear acceleration of {@code endEffector.getBodyFixedFrame()}'s origin.
    *           Modified.
    */
   public void changeFrameAndSet(FramePoint3D desiredPosition, FrameVector3D desiredLinearVelocity, FrameVector3D feedForwardLinearAcceleration)
   {
      desiredPosition.changeFrame(worldFrame);
      desiredLinearVelocity.changeFrame(worldFrame);
      feedForwardLinearAcceleration.changeFrame(worldFrame);

      desiredPositionInWorld.set(desiredPosition);
      desiredLinearVelocityInWorld.set(desiredLinearVelocity);
      feedForwardLinearAccelerationInWorld.set(feedForwardLinearAcceleration);
   }

   /**
    * Change the reference frame of the given data such that it is expressed in
    * {@link ReferenceFrame#getWorldFrame()}. The data will be used for the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredOrientation describes the orientation that the {@code controlFrame} should
    *           reach. It does NOT describe the desired orientation of
    *           {@code endEffector.getBodyFixedFrame()}. Modified.
    * @param desiredAngularVelocity describes the desired linear velocity of {@code controlFrame}
    *           with respect to the {@code base}. It is equivalent to the desired angular velocity
    *           of {@code endEffector.getBodyFixedFrame()}. Modified.
    * @param feedForwardAngularAcceleration describes the desired linear acceleration of
    *           {@code controlFrame} with respect to the {@code base}. It is equivalent to the
    *           desired angular acceleration of {@code endEffector.getBodyFixedFrame()}. Modified.
    */
   public void changeFrameAndSet(FrameQuaternion desiredOrientation, FrameVector3D desiredAngularVelocity, FrameVector3D feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      feedForwardAngularAcceleration.changeFrame(worldFrame);

      desiredOrientationInWorld.set(desiredOrientation);
      desiredAngularVelocityInWorld.set(desiredAngularVelocity);
      feedForwardAngularAccelerationInWorld.set(feedForwardAngularAcceleration);
   }

   /**
    * Zeroes the offset of the {@code controlFrame} such that after calling this method
    * {@code controlFrame == endEffector.getBodyFixedFrame()}.
    */
   public void resetControlFrame()
   {
      controlFrameOriginInEndEffectorFrame.setToZero();
      controlFrameOrientationInEndEffectorFrame.setToZero();
   }

   /**
    * Sets the position of the {@code controlFrame}'s origin with respect to the
    * {@code endEffector.getBodyFixedFrame()}. The {@code controlFrame} will have the same
    * orientation as the end-effector body-fixed frame.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame}
    * to the given desired position and orientation.
    * </p>
    *
    * @param position the position of the {@code controlFrame}'s origin. Not modified.
    * @throws ReferenceFrameMismatchException if any of the {@code position} is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setControlFrameFixedInEndEffector(FramePoint3D position)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      controlFrameOriginInEndEffectorFrame.set(position);
      controlFrameOrientationInEndEffectorFrame.setToZero();
   }

   /**
    * Sets the position and orientation of the {@code controlFrame} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame}
    * to the given desired position and orientation.
    * </p>
    *
    * @param position the position of the {@code controlFrame}'s origin. Not modified.
    * @param orientation the orientation of the {@code controlFrame}. Not modified.
    * @throws ReferenceFrameMismatchException if any of the two arguments is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setControlFrameFixedInEndEffector(FramePoint3D position, FrameQuaternion orientation)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      orientation.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      controlFrameOriginInEndEffectorFrame.set(position);
      controlFrameOrientationInEndEffectorFrame.set(orientation);
   }

   /**
    * Changes the argument frame to {@code endEffector.getBodyFixedFrame()} and the sets the
    * position and orientation of the {@code controlFrame} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame}
    * to the given desired position and orientation.
    * </p>
    *
    * @param position the position of the {@code controlFrame}'s origin. Modified.
    * @param orientation the orientation of the {@code controlFrame}. Modified.
    */
   public void changeFrameAndSetControlFrameFixedInEndEffector(FramePoint3D position, FrameQuaternion orientation)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.changeFrame(endEffector.getBodyFixedFrame());
      orientation.changeFrame(endEffector.getBodyFixedFrame());
      controlFrameOriginInEndEffectorFrame.set(position);
      controlFrameOrientationInEndEffectorFrame.set(orientation);
   }

   /**
    * Sets the pose of the {@code controlFrame} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame}
    * to the given desired position and orientation.
    * </p>
    *
    * @param pose the pose of the {@code controlFrame}. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setControlFrameFixedInEndEffector(FramePose pose)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      pose.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      pose.getPose(controlFrameOriginInEndEffectorFrame, controlFrameOrientationInEndEffectorFrame);
   }

   /**
    * Changes the argument frame to {@code endEffector.getBodyFixeFrame()} and then sets the pose of
    * the {@code controlFrame} with respect to the {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame}
    * to the given desired position and orientation.
    * </p>
    *
    * @param pose the of the {@code controlFrame}. Modified.
    */
   public void changeFrameAndSetControlFrameFixedInEndEffector(FramePose pose)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      pose.changeFrame(endEffector.getBodyFixedFrame());
      pose.getPose(controlFrameOriginInEndEffectorFrame, controlFrameOrientationInEndEffectorFrame);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom of the end-effector are to be controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. It is initialized such that the controller will by default control
    * all the end-effector DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the
    * selection frame is equal to the control frame.
    * </p>
    *
    * @param selectionMatrix the selection matrix to copy data from. Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix)
   {
      spatialAccelerationCommand.setSelectionMatrix(selectionMatrix);
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
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weightMatrix weight matrix holding the weights to use for each component of the desired
    *           acceleration. Not modified.
    */
   public void setWeightMatrixForSolver(WeightMatrix6D weightMatrix)
   {
      spatialAccelerationCommand.setWeightMatrix(weightMatrix);
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
    * @param linear the weights to use for the linear part of this command. Not modified.
    */
   public void setWeightsForSolver(Vector3DReadOnly angular, Vector3DReadOnly linear)
   {
      spatialAccelerationCommand.setWeights(angular, linear);
   }

   public void getIncludingFrame(FramePoint3D desiredPositionToPack, FrameQuaternion desiredOrientationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
   }

   public void getIncludingFrame(FramePoint3D desiredPositionToPack, FrameVector3D desiredLinearVelocityToPack, FrameVector3D feedForwardLinearAccelerationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredLinearVelocityToPack.setIncludingFrame(worldFrame, desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationToPack.setIncludingFrame(worldFrame, feedForwardLinearAccelerationInWorld);
   }

   public void getIncludingFrame(FrameQuaternion desiredOrientationToPack, FrameVector3D desiredAngularVelocityToPack,
                                 FrameVector3D feedForwardAngularAccelerationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
      desiredAngularVelocityToPack.setIncludingFrame(worldFrame, desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationToPack.setIncludingFrame(worldFrame, feedForwardAngularAccelerationInWorld);
   }

   public void getControlFramePoseIncludingFrame(FramePoint3D position, FrameQuaternion orientation)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.setIncludingFrame(endEffector.getBodyFixedFrame(), controlFrameOriginInEndEffectorFrame);
      orientation.setIncludingFrame(endEffector.getBodyFixedFrame(), controlFrameOrientationInEndEffectorFrame);
   }

   public RigidBody getBase()
   {
      return spatialAccelerationCommand.getBase();
   }

   public RigidBody getEndEffector()
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

   public PIDSE3Gains getGains()
   {
      return gains;
   }

   public ReferenceFrame getAngularGainsFrame()
   {
      return angularGainsFrame;
   }

   public ReferenceFrame getLinearGainsFrame()
   {
      return linearGainsFrame;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.TASKSPACE;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": ";
      ret += "base = " + spatialAccelerationCommand.getBaseName() + ", ";
      ret += "endEffector = " + spatialAccelerationCommand.getEndEffectorName() + ", ";
      ret += "position = " + desiredPositionInWorld + ", orientation = " + desiredOrientationInWorld.toStringAsYawPitchRoll();
      return ret;
   }
}
