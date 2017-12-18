package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   /** The end-effector's desired orientation expressed in world frame. */
   private final Quaternion desiredOrientationInWorld = new Quaternion();
   /** The end-effector's desired angular velocity expressed in world frame. */
   private final Vector3D desiredAngularVelocityInWorld = new Vector3D();
   /** The feed-forward to be used for the end-effector. Useful to improve tracking performance. */
   private final Vector3D feedForwardAngularAccelerationInWorld = new Vector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final PID3DGains gains = new DefaultPID3DGains();
   /** This is the reference frame in which the angular part of the gains are to be applied. If {@code null}, it is applied in the control frame. */
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
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect
    * to the control base frame.
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
      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      desiredAngularVelocityInWorld.set(other.desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationInWorld.set(other.feedForwardAngularAccelerationInWorld);
      gains.set(other.gains);

      spatialAccelerationCommand.set(other.spatialAccelerationCommand);
      controlBaseFrame = other.controlBaseFrame;
   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be to control the end-effector.
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
    * @param angularGainsFrame the reference frame to use for the orientation gains.
    */
   public void setGainsFrame(ReferenceFrame angularGainsFrame)
   {
      this.angularGainsFrame = angularGainsFrame;
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * The desired angular velocity and feed-forward angular acceleration are set to zero.
    * </p>
    *
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Not modified.
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
    *
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Not modified.
    * @param desiredAngularVelocity describes the desired linear velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not
    *           modified.
    * @param feedForwardAngularAcceleration describes the desired linear acceleration of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not
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
    *
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Modified.
    * @param desiredAngularVelocity describes the desired linear velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Modified.
    * @param feedForwardAngularAcceleration describes the desired linear acceleration of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Modified.
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
    * This specifies that the 3 rotational degrees of freedom of the end-effector are to be
    * controlled.
    */
   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixForAngularControl();
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
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angular the weights to use for the angular part of this command. Not modified.
    * @param linear the weight to use for the linear part of this command. Not modified.
    */
   public void setWeightsForSolver(Vector3DReadOnly weight)
   {
      spatialAccelerationCommand.setAngularWeights(weight);
      spatialAccelerationCommand.setLinearWeightsToZero();
   }

   public void getIncludingFrame(FrameQuaternion desiredOrientationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
   }

   public void getIncludingFrame(FrameQuaternion desiredOrientationToPack, FrameVector3D desiredAngularVelocityToPack,
                                 FrameVector3D feedForwardAngularAccelerationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
      desiredAngularVelocityToPack.setIncludingFrame(worldFrame, desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationToPack.setIncludingFrame(worldFrame, feedForwardAngularAccelerationInWorld);
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
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": ";
      ret += "base = " + spatialAccelerationCommand.getBaseName() + ", ";
      ret += "endEffector = " + spatialAccelerationCommand.getEndEffectorName() + ", ";
      ret += "orientation = " + desiredOrientationInWorld.toStringAsYawPitchRoll();
      return ret;
   }
}
