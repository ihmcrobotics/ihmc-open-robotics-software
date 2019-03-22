package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link SpatialFeedbackControlCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link SpatialFeedbackControlCommand} is to notify the feedback controller
 * dedicated to control the end-effector provided in {@link #set(RigidBodyBasics, RigidBodyBasics)}
 * that it is requested to run during the next control tick.
 * </p>
 * <p>
 * From control tick to control tick each feedback controller can be entirely configured or
 * reconfigured, and enabled (by submitting a command) or disabled (by NOT submitting a command).
 * </p>
 * <p>
 * All the data contained in this command is expressed in root to ensure that the feedback
 * controller can properly interpret it.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class SpatialFeedbackControlCommand implements FeedbackControlCommand<SpatialFeedbackControlCommand>
{
   private final FramePose3D controlFramePoseInEndEffectorFrame = new FramePose3D();

   /** Represents the expected control mode to execute this command. */
   private WholeBodyControllerCoreMode controlMode = null;
   /** The desired orientation to use in the feedback controller. */
   private final FrameQuaternion referenceOrientationInRootFrame = new FrameQuaternion();
   /** The desired position to use in the feedback controller. */
   private final FramePoint3D referencePositionInRootFrame = new FramePoint3D();

   /** The desired or (IK) feed-forward angular velocity to use in the feedback controller. */
   private final FrameVector3D referenceAngularVelocityInRootFrame = new FrameVector3D();
   /** The desired or (IK) feed-forward linear velocity to use in the feedback controller. */
   private final FrameVector3D referenceLinearVelocityInRootFrame = new FrameVector3D();

   /** The (ID) feed-forward angular acceleration to use in the feedback controller. */
   private final FrameVector3D referenceAngularAccelerationInRootFrame = new FrameVector3D();
   /** The (ID) feed-forward linear acceleration to use in the feedback controller. */
   private final FrameVector3D referenceLinearAccelerationInRootFrame = new FrameVector3D();

   /** The (VMC) feed-forward torque to use in the feedback controller. */
   private final FrameVector3D referenceTorqueInRootFrame = new FrameVector3D();
   /** The (VMC) feed-forward force to use in the feedback controller. */
   private final FrameVector3D referenceForceInRootFrame = new FrameVector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
   /**
    * This is the reference frame in which the angular part of the gains are to be applied. If
    * {@code null}, it is applied in the control frame.
    */
   private ReferenceFrame angularGainsFrame = null;
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
      controlFramePoseInEndEffectorFrame.setIncludingFrame(other.controlFramePoseInEndEffectorFrame);
      controlMode = other.controlMode;
      referenceOrientationInRootFrame.setIncludingFrame(other.referenceOrientationInRootFrame);
      referencePositionInRootFrame.setIncludingFrame(other.referencePositionInRootFrame);
      referenceAngularVelocityInRootFrame.setIncludingFrame(other.referenceAngularVelocityInRootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(other.referenceLinearVelocityInRootFrame);
      referenceAngularAccelerationInRootFrame.setIncludingFrame(other.referenceAngularAccelerationInRootFrame);
      referenceLinearAccelerationInRootFrame.setIncludingFrame(other.referenceLinearAccelerationInRootFrame);
      referenceTorqueInRootFrame.setIncludingFrame(other.referenceTorqueInRootFrame);
      referenceForceInRootFrame.setIncludingFrame(other.referenceForceInRootFrame);

      gains.set(other.gains);
      angularGainsFrame = other.angularGainsFrame;
      linearGainsFrame = other.linearGainsFrame;
      spatialAccelerationCommand.set(other.spatialAccelerationCommand);
      controlBaseFrame = other.controlBaseFrame;

   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be used to control the end-effector.
    * </p>
    *
    * @param base the rigid-body located right before the first joint to be used for controlling the
    *           end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBodyBasics base, RigidBodyBasics endEffector)
   {
      spatialAccelerationCommand.set(base, endEffector);
      resetControlFrame();
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
    * Sets whether or not to scale the weights on the joints below the intermediate base defined by
    * {@link #setPrimaryBase(RigidBodyBasics)}. Indicates that we would like to custom scale the
    * weights on the joints in the kinematic chain below the {@code primaryBase} when controlling the
    * {@code endEffector}.
    * <p>
    * If false, as is the case in the default setting, the controller uses the default scaling factor
    * {@link us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator#secondaryTaskJointsWeight}.
    * </p>
    * <p>
    * If true, the controller uses the custom defined scaling factor
    * {@code secondaryTaskJointWeightScale} to scale the weights before the {@code primaryBase} in the
    * kinematic chain between the {@code base} and {@code endEffector}.
    * </p>
    * <p>
    * A scale factor greater than 1.0 indicates that it is desired to use the joints in the kinematic
    * chain between {@code base} and {@code primaryBase} to control the {@code endEffector} more than
    * the joints between the {@code primaryBase} and the {@code endEffector}. For example, this can be
    * used to say that we would prefer to use the pelvis to control the foot acceleration than the leg
    * joints.
    * </p>
    * <p>
    * A scale factor less than 1.0 indicates that it is desired to use the joints in the kinematic
    * chain between {@code primaryBase} and {@code endEffector} to control the {@code endEffector} more
    * than the joints between the {@code base} and the {@code primaryBase}. For example, this can be
    * used to say that we would prefer to use the leg joints to control the foot acceleration than the
    * pelvis.
    * </p>
    *
    * @param scaleSecondaryTaskJointWeight whether or not to use a custom scaling factor on the joints
    *           below the primary base. Optional.
    * @param secondaryTaskJointWeightScale custom scaling factor for the joints below the primary base.
    *           Optional.
    */
   public void setScaleSecondaryTaskJointWeight(boolean scaleSecondaryTaskJointWeight, double secondaryTaskJointWeightScale)
   {
      spatialAccelerationCommand.setScaleSecondaryTaskJointWeight(scaleSecondaryTaskJointWeight, secondaryTaskJointWeightScale);
   }

   /**
    * Resets the secondary task joint weight scaling factor on the joints below the {@code primaryBase}
    * to its default value.
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
    * If a reference frame is {@code null}, the corresponding gains will be applied in the control
    * frame.
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
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code controlFrame} should reach. Not modified.
    * @param feedForwardAngularVelocity the feed-forward angular velocity of {@code controlFrame} with
    *           respect to the {@code base}. Not modified.
    */
   public void setInverseKinematics(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly feedForwardAngularVelocity)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      ReferenceFrame rootFrame = desiredOrientation.getReferenceFrame().getRootFrame();
      referenceOrientationInRootFrame.setIncludingFrame(desiredOrientation);
      referenceOrientationInRootFrame.changeFrame(rootFrame);
      referenceAngularVelocityInRootFrame.setIncludingFrame(feedForwardAngularVelocity);
      referenceAngularVelocityInRootFrame.changeFrame(rootFrame);
      referenceAngularAccelerationInRootFrame.setToZero(rootFrame);
      referenceTorqueInRootFrame.setToZero(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for inverse kinematics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse kinematics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredPosition the position the {@code controlFrame} should reach. Not modified.
    * @param feedForwardLinearVelocity the feed-forward linear velocity of the {@code controlFrame}'s
    *           origin with respect to the {@code base}. Not modified.
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
    * Configures this feedback command's inputs for inverse kinematics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse kinematics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code controlFrame} should reach. Not modified.
    * @param desiredPosition the position the {@code controlFrame} should reach. Not modified.
    * @param feedForwardAngularVelocity the feed-forward angular velocity of {@code controlFrame} with
    *           respect to the {@code base}. This term is used to improve tracking performance. Not
    *           modified.
    * @param feedForwardLinearVelocity the feed-forward linear velocity of the {@code controlFrame}'s
    *           origin with respect to the {@code base}. Not modified.
    */
   public void setInverseKinematics(FrameQuaternionReadOnly desiredOrientation, FramePoint3DReadOnly desiredPosition,
                                    FrameVector3DReadOnly feedForwardAngularVelocity, FrameVector3DReadOnly feedForwardLinearVelocity)
   {
      setInverseKinematics(desiredOrientation, feedForwardAngularVelocity);
      setInverseKinematics(desiredPosition, feedForwardLinearVelocity);
   }

   /**
    * Configures this feedback command's inputs for inverse kinematics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse kinematics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredPose the pose the {@code controlFrame} should reach. Not modified.
    * @param feedForwardVelocity the feed-forward angular & linear velocity of {@code controlFrame}
    *           with respect to the {@code base}. Not modified.
    */
   public void setInverseKinematics(FramePose3DReadOnly desiredPose, SpatialVectorReadOnly feedForwardVelocity)
   {
      setInverseKinematics(desiredPose.getOrientation(), feedForwardVelocity.getAngularPart());
      setInverseKinematics(desiredPose.getPosition(), feedForwardVelocity.getLinearPart());
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse dynamics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code controlFrame} should reach. Not modified.
    * @param desiredAngularVelocity the desired angular velocity of {@code controlFrame} with respect
    *           to the {@code base}. Not modified.
    * @param feedForwardAngularAcceleration the feed-forward angular acceleration of
    *           {@code controlFrame} with respect to the {@code base}. Not modified.
    */
   public void setInverseDynamics(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly desiredAngularVelocity,
                                  FrameVector3DReadOnly feedForwardAngularAcceleration)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      ReferenceFrame rootFrame = desiredOrientation.getReferenceFrame().getRootFrame();
      referenceOrientationInRootFrame.setIncludingFrame(desiredOrientation);
      referenceOrientationInRootFrame.changeFrame(rootFrame);
      referenceAngularVelocityInRootFrame.setIncludingFrame(desiredAngularVelocity);
      referenceAngularVelocityInRootFrame.changeFrame(rootFrame);
      referenceAngularAccelerationInRootFrame.setIncludingFrame(feedForwardAngularAcceleration);
      referenceAngularAccelerationInRootFrame.changeFrame(rootFrame);
      referenceTorqueInRootFrame.setToZero(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse dynamics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredPosition the position that the {@code controlFrame} should reach. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the {@code controlFrame}'s origin
    *           with respect to the {@code base}. Not modified.
    * @param feedForwardLinearAcceleration the feed-forward linear acceleration of the
    *           {@code controlFrame}'s origin with respect to the {@code base}. Not modified.
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
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse dynamics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    * 
    * @param desiredOrientation the orientation the {@code controlFrame} should reach. Not modified.
    * @param desiredPosition the position that the {@code controlFrame} should reach. Not modified.
    * @param desiredAngularVelocity the desired angular velocity of {@code controlFrame} with respect
    *           to the {@code base}. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the {@code controlFrame}'s origin
    *           with respect to the {@code base}. Not modified.
    * @param feedForwardAngularAcceleration the feed-forward angular acceleration of
    *           {@code controlFrame} with respect to the {@code base}. Not modified.
    * @param feedForwardLinearAcceleration the feed-forward linear acceleration of the
    *           {@code controlFrame}'s origin with respect to the {@code base}. Not modified.
    */
   public void setInverseDynamics(FrameQuaternionReadOnly desiredOrientation, FramePoint3DReadOnly desiredPosition,
                                  FrameVector3DReadOnly desiredAngularVelocity, FrameVector3DReadOnly desiredLinearVelocity,
                                  FrameVector3DReadOnly feedForwardAngularAcceleration, FrameVector3DReadOnly feedForwardLinearAcceleration)
   {
      setInverseDynamics(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse dynamics.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    * 
    * @param desiredPose the pose that the {@code controlFrame} should reach. Not modified.
    * @param desiredVelocity the desired angular & linear velocity of {@code controlFrame} with respect
    *           to the {@code base}. Not modified.
    * @param feedForwardAcceleration the feed-forward angular & linear acceleration of
    *           {@code controlFrame} with respect to the {@code base}. Not modified.
    */
   public void setInverseDynamics(FramePose3DReadOnly desiredPose, SpatialVectorReadOnly desiredVelocity, SpatialVectorReadOnly feedForwardAcceleration)
   {
      setInverseDynamics(desiredPose.getOrientation(), desiredVelocity.getAngularPart(), feedForwardAcceleration.getAngularPart());
      setInverseDynamics(desiredPose.getPosition(), desiredVelocity.getLinearPart(), feedForwardAcceleration.getLinearPart());
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredOrientation the orientation the {@code controlFrame} should reach. Not modified.
    * @param desiredAngularVelocity the desired angular velocity of {@code controlFrame} with respect
    *           to the {@code base}. Not modified.
    * @param feedForwardTorque the feed-forward torque to exert at {@code controlFrame}. Not modified.
    */
   public void setVirtualModelControl(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly desiredAngularVelocity,
                                      FrameVector3DReadOnly feedForwardTorque)
   {
      setControlMode(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
      ReferenceFrame rootFrame = desiredOrientation.getReferenceFrame().getRootFrame();
      referenceOrientationInRootFrame.setIncludingFrame(desiredOrientation);
      referenceOrientationInRootFrame.changeFrame(rootFrame);
      referenceAngularVelocityInRootFrame.setIncludingFrame(desiredAngularVelocity);
      referenceAngularVelocityInRootFrame.changeFrame(rootFrame);
      referenceTorqueInRootFrame.setIncludingFrame(feedForwardTorque);
      referenceTorqueInRootFrame.changeFrame(rootFrame);
      referenceAngularAccelerationInRootFrame.setToZero(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * @param desiredPosition the position that the {@code controlFrame} should reach. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the {@code controlFrame}'s origin
    *           with respect to the {@code base}. Not modified.
    * @param feedForwardForce the feed-forward force to exert at {@code controlFrame}. Not modified.
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
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * 
    * @param desiredOrientation the orientation the {@code controlFrame} should reach. Not modified.
    * @param desiredPosition the position that the {@code controlFrame} should reach. Not modified.
    * @param desiredAngularVelocity the desired angular velocity of {@code controlFrame} with respect
    *           to the {@code base}. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the {@code controlFrame}'s origin
    *           with respect to the {@code base}. Not modified.
    * @param feedForwardTorque the feed-forward torque to exert at {@code controlFrame}. Not modified.
    * @param feedForwardForce the feed-forward force to exert at {@code controlFrame}. Not modified.
    */
   public void setVirtualModelControl(FrameQuaternionReadOnly desiredOrientation, FramePoint3DReadOnly desiredPosition,
                                      FrameVector3DReadOnly desiredAngularVelocity, FrameVector3DReadOnly desiredLinearVelocity,
                                      FrameVector3DReadOnly feedForwardTorque, FrameVector3DReadOnly feedForwardForce)
   {
      setVirtualModelControl(desiredOrientation, desiredAngularVelocity, feedForwardTorque);
      setVirtualModelControl(desiredPosition, desiredLinearVelocity, feedForwardForce);
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    * <p>
    * WARNING: The information provided has to be relevant to the {@code controlFrame} provided.
    * </p>
    *
    * 
    * @param desiredPose the pose that the {@code controlFrame} should reach. Not modified.
    * @param desiredVelocity the desired angular & linear velocity of {@code controlFrame} with respect
    *           to the {@code base}. Not modified.
    * @param feedForwardEffort the feed-forward torque & force to exert at {@code controlFrame}. Not
    *           modified.
    */
   public void setVirtualModelControl(FramePose3DReadOnly desiredPose, SpatialVectorReadOnly desiredVelocity, SpatialVectorReadOnly feedForwardEffort)
   {
      setVirtualModelControl(desiredPose.getOrientation(), desiredVelocity.getAngularPart(), feedForwardEffort.getAngularPart());
      setVirtualModelControl(desiredPose.getPosition(), desiredVelocity.getLinearPart(), feedForwardEffort.getLinearPart());
   }

   /**
    * Zeroes the offset of the {@code controlFrame} such that after calling this method
    * {@code controlFrame == endEffector.getBodyFixedFrame()}.
    */
   public void resetControlFrame()
   {
      controlFramePoseInEndEffectorFrame.setToZero(getEndEffector().getBodyFixedFrame());
   }

   /**
    * Sets the position of the {@code controlFrame}'s origin with respect to the
    * {@code endEffector.getBodyFixedFrame()}. The {@code controlFrame} will have the same orientation
    * as the end-effector body-fixed frame.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired position and orientation.
    * </p>
    *
    * @param position the position of the {@code controlFrame}'s origin. Not modified.
    * @throws ReferenceFrameMismatchException if any of the {@code position} is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setControlFrameFixedInEndEffector(FramePoint3DReadOnly position)
   {
      RigidBodyBasics endEffector = spatialAccelerationCommand.getEndEffector();
      position.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      controlFramePoseInEndEffectorFrame.setToZero(endEffector.getBodyFixedFrame());
      controlFramePoseInEndEffectorFrame.setPosition(position);
   }

   /**
    * Sets the position and orientation of the {@code controlFrame} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired position and orientation.
    * </p>
    *
    * @param position the position of the {@code controlFrame}'s origin. Not modified.
    * @param orientation the orientation of the {@code controlFrame}. Not modified.
    * @throws ReferenceFrameMismatchException if any of the two arguments is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setControlFrameFixedInEndEffector(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      RigidBodyBasics endEffector = spatialAccelerationCommand.getEndEffector();
      position.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      orientation.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      controlFramePoseInEndEffectorFrame.setIncludingFrame(position, orientation);
   }

   /**
    * Sets the pose of the {@code controlFrame} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired position and orientation.
    * </p>
    *
    * @param pose the pose of the {@code controlFrame}. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setControlFrameFixedInEndEffector(FramePose3DReadOnly pose)
   {
      RigidBodyBasics endEffector = spatialAccelerationCommand.getEndEffector();
      pose.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      controlFramePoseInEndEffectorFrame.setIncludingFrame(pose);
   }

   /**
    * Sets the pose of the {@code controlFrame} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code controlFrame} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired position and orientation.
    * </p>
    *
    * @param poseInBodyFrame the pose of the {@code controlFrame}. Not modified.
    */
   public void setControlFrameFixedInEndEffector(RigidBodyTransform poseInBodyFrame)
   {
      controlFramePoseInEndEffectorFrame.setIncludingFrame(getEndEffector().getBodyFixedFrame(), poseInBodyFrame);
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
    * Sets the weight to use in the optimization problem.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angularWeight the weight value to use for the angular portion of this command.
    * @param linearWeight the weight value to use for the linear portion of this command.
    */
   public void setWeightsForSolver(double angularWeight, double linearWeight)
   {
      spatialAccelerationCommand.setWeight(angularWeight, linearWeight);
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

   /**
    * Sets the weights to use in the optimization problem for each individual linear degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param linear the weights to use for the linear part of this command. Not modified.
    */
   public void setLinearWeightsForSolver(Vector3DReadOnly linear)
   {
      spatialAccelerationCommand.setLinearWeights(linear);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual angular degree of
    * freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angular the weights to use for the linear part of this command. Not modified.
    */
   public void setAngularWeightsForSolver(Vector3DReadOnly angular)
   {
      spatialAccelerationCommand.setAngularWeights(angular);
   }

   public FramePose3DBasics getControlFramePose()
   {
      return controlFramePoseInEndEffectorFrame;
   }

   public void getControlFramePoseIncludingFrame(FramePoint3DBasics position, FrameQuaternionBasics orientation)
   {
      position.setIncludingFrame(controlFramePoseInEndEffectorFrame.getPosition());
      orientation.setIncludingFrame(controlFramePoseInEndEffectorFrame.getOrientation());
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
      return referenceOrientationInRootFrame;
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
      return referenceAngularVelocityInRootFrame;
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
    * Gets the reference angular acceleration to use in the feedback controller.
    * <p>
    * It is used in the inverse dynamics mode as a feed-forward term.
    * </p>
    * 
    * @return the reference angular acceleration.
    */
   public FrameVector3DBasics getReferenceAngularAcceleration()
   {
      return referenceAngularAccelerationInRootFrame;
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
    * Gets the reference torque to use in the feedback controller.
    * <p>
    * It is used in the virtual control mode as a feed-forward term.
    * </p>
    * 
    * @return the reference torque.
    */
   public FrameVector3DBasics getReferenceTorque()
   {
      return referenceTorqueInRootFrame;
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
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof SpatialFeedbackControlCommand)
      {
         SpatialFeedbackControlCommand other = (SpatialFeedbackControlCommand) object;

         if (controlMode != other.controlMode)
            return false;
         if (!controlFramePoseInEndEffectorFrame.equals(other.controlFramePoseInEndEffectorFrame))
            return false;
         if (!referenceOrientationInRootFrame.equals(other.referenceOrientationInRootFrame))
            return false;
         if (!referencePositionInRootFrame.equals(other.referencePositionInRootFrame))
            return false;
         if (!referenceAngularVelocityInRootFrame.equals(other.referenceAngularVelocityInRootFrame))
            return false;
         if (!referenceLinearVelocityInRootFrame.equals(other.referenceLinearVelocityInRootFrame))
            return false;
         if (!referenceAngularAccelerationInRootFrame.equals(other.referenceAngularAccelerationInRootFrame))
            return false;
         if (!referenceLinearAccelerationInRootFrame.equals(other.referenceLinearAccelerationInRootFrame))
            return false;
         if (!referenceTorqueInRootFrame.equals(other.referenceTorqueInRootFrame))
            return false;
         if (!referenceForceInRootFrame.equals(other.referenceForceInRootFrame))
            return false;
         if (!gains.equals(other.gains))
            return false;
         if (linearGainsFrame != other.linearGainsFrame)
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
      ret += "position = " + referencePositionInRootFrame + ", orientation = " + referenceOrientationInRootFrame.toStringAsYawPitchRoll();
      return ret;
   }
}
