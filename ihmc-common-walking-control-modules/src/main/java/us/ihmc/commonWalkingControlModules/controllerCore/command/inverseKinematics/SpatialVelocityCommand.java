package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link SpatialVelocityCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link SpatialVelocityCommand} is to notify the inverse kinematics
 * optimization module that the given end-effector is to track a desired velocity during the next
 * control tick.
 * </p>
 * <p>
 * It is usually the result of the {@link SpatialFeedbackController}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class SpatialVelocityCommand implements InverseKinematicsCommand<SpatialVelocityCommand>
{
   /** Defines the reference frame of interest. It is attached to the end-effector. */
   private final FramePose controlFramePose = new FramePose();

   /**
    * It defines the desired linear velocity of the origin of the control frame, with respect to the
    * base. The vector is expressed in the control frame.
    */
   private final Vector3D desiredLinearVelocity = new Vector3D();
   /**
    * It defines the desired angular velocity of the control frame, with respect to the base. The
    * vector is expressed in the control frame.
    */
   private final Vector3D desiredAngularVelocity = new Vector3D();

   /**
    * The Weight matrix describes the qp weights used for optimization. All weights are initially set to NaN. If the weights are NaN the controller will use the default weights.
    * A higher weight means a higher priority of this task.
    */
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. It is initialized such that the controller will by default control
    * all the end-effector DoFs.
    * <p>
    * If the selection frame is not set, it is assumed that the selection frame is equal to the
    * control frame.
    * </p>
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   /**
    * The command type describes the nature of the constraint that is being imposed on the the end-effector
    * <p> If the command type is not set it defaults to a {@code ConstraintType#OBJECTIVE} constraint
    */
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   /**
    * The base is the rigid-body located right before the first joint to be used for controlling the
    * end-effector.
    */
   private RigidBody base;
   /** The end-effector is the rigid-body to be controlled. */
   private RigidBody endEffector;
   /**
    * Intermediate base located between the {@code base} and {@code endEffector}. It can be null.
    */
   private RigidBody optionalPrimaryBase;

   private String baseName;
   private String endEffectorName;
   private String optionalPrimaryBaseName;

   /**
    * Flag to indicate whether or not to custom scale the weights below the intermediate base
    * {@code optionalPrimaryBase} to control against, as opposed to using the default weight in
    * {@link us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator#secondaryTaskJointsWeight}.
    */
   private boolean scaleSecondaryTaskJointWeight = false;

   /**
    * Scale factor to apply to the weights on the task below the {@code optionalPrimaryBase}.
    * This weight replaces the scale factor in
    * {@link us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator#secondaryTaskJointsWeight}.
    */
   private double secondaryTaskJointWeightScale = 1.0;

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public SpatialVelocityCommand()
   {
      setAsHardEqualityConstraint();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(SpatialVelocityCommand other)
   {
      setWeightMatrix(other.getWeightMatrix());

      selectionMatrix.set(other.selectionMatrix);
      base = other.getBase();
      endEffector = other.getEndEffector();
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;

      optionalPrimaryBase = other.optionalPrimaryBase;
      optionalPrimaryBaseName = other.optionalPrimaryBaseName;
      scaleSecondaryTaskJointWeight = other.scaleSecondaryTaskJointWeight;
      secondaryTaskJointWeightScale = other.secondaryTaskJointWeightScale;

      controlFramePose.setPoseIncludingFrame(endEffector.getBodyFixedFrame(), other.controlFramePose.getPosition(), other.controlFramePose.getOrientation());
      desiredAngularVelocity.set(other.desiredAngularVelocity);
      desiredLinearVelocity.set(other.desiredLinearVelocity);
   }

   /**
    * Copies all the fields of the given {@link SpatialAccelerationCommand} into this except for the
    * spatial acceleration.
    * 
    * @param command the command to copy the properties from. Not modified.
    */
   public void setProperties(SpatialAccelerationCommand command)
   {
      setWeightMatrix(command.getWeightMatrix());

      command.getSelectionMatrix(selectionMatrix);
      base = command.getBase();
      endEffector = command.getEndEffector();
      baseName = command.getBaseName();
      endEffectorName = command.getEndEffectorName();

      optionalPrimaryBase = command.getPrimaryBase();
      optionalPrimaryBaseName = command.getPrimaryBaseName();
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
      this.base = base;
      this.endEffector = endEffector;

      baseName = base.getName();
      endEffectorName = endEffector.getName();
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
    * @param primaryBase the rigid-body to use as the primary base. Optional.
    */
   public void setPrimaryBase(RigidBody primaryBase)
   {
      optionalPrimaryBase = primaryBase;
      optionalPrimaryBaseName = primaryBase.getName();
   }

   /**
    * Indicates that we would like to custom scale the weights on the joints in the kinematic chain
    * below the {@code primaryBase} when controlling the {@code endEffector}.
    *
    * @param scaleSecondaryTaskJointWeight whether or not to use a custom scaling factor on the joints
    *                                      below the primary base. Optional.
    * @param secondaryTaskJointWeightScale custom scaling factor for the joints below the primary base.
    *                                      Optional.
    */
   public void setScaleSecondaryTaskJointWeight(boolean scaleSecondaryTaskJointWeight, double secondaryTaskJointWeightScale)
   {
      this.scaleSecondaryTaskJointWeight = scaleSecondaryTaskJointWeight;
      this.secondaryTaskJointWeightScale = secondaryTaskJointWeightScale;
   }

   /**
    * Sets the desired velocity to submit for the optimization to zero.
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be
    * attached to the end-effector. For instance, when controlling a foot, the {@code controlFrame}
    * should be located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    */
   public void setSpatialVelocityToZero(ReferenceFrame controlFrame)
   {
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredAngularVelocity.setToZero();
      desiredLinearVelocity.setToZero();
   }

   /**
    * Sets the desired velocity to submit for the optimization.
    * <p>
    * It is important that the spatial velocity describes the velocity of the control frame
    * especially if only part of the angular velocity is to be controlled.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be
    * attached to the end-effector. For instance, when controlling a foot, the {@code controlFrame}
    * should be located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredSpatialVelocity the desired end-effector velocity with respect to the base and
    *           expressed in the control frame.
    * @throws ReferenceFrameMismatchException if the {@code desiredSpatialVelocity} is not setup as
    *            follows: {@code bodyFrame = endEffector.getBodyFixedFrame()},
    *            {@code baseFrame = base.getBodyFixedFrame()},
    *            {@code expressedInFrame = controlFrame}.
    */
   public void setSpatialVelocity(ReferenceFrame controlFrame, Twist desiredSpatialVelocity)
   {
      desiredSpatialVelocity.getBodyFrame().checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      desiredSpatialVelocity.getBaseFrame().checkReferenceFrameMatch(base.getBodyFixedFrame());
      desiredSpatialVelocity.getExpressedInFrame().checkReferenceFrameMatch(controlFrame);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredSpatialVelocity.getAngularPart(desiredAngularVelocity);
      desiredSpatialVelocity.getLinearPart(desiredLinearVelocity);
   }

   /**
    * Sets the desired velocity to submit for the optimization.
    * <p>
    * The {@code desiredAngularVelocity} has to define the desired angular velocity of the
    * end-effector with respect to the base. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The {@code desiredLinearVelocity} has to defined the desired linear velocity of the origin of
    * the {@code controlFrame}. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be
    * attached to the end-effector. For instance, when controlling a foot, the {@code controlFrame}
    * should be located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredAngularVelocity the desired angular velocity of the end-effector with respect to
    *           the base. Not modified.
    * @param desiredLinearVelocity the desired linear velocity of the origin of the control frame
    *           with respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredAngularVelocity} or
    *            {@code desiredLineaerVelocitys} is not expressed in control frame.
    */
   public void setSpatialVelocity(ReferenceFrame controlFrame, FrameVector3D desiredAngularVelocity, FrameVector3D desiredLinearVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularVelocity);
      controlFrame.checkReferenceFrameMatch(desiredLinearVelocity);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      this.desiredAngularVelocity.set(desiredAngularVelocity);
      this.desiredLinearVelocity.set(desiredLinearVelocity);
   }

   /**
    * Sets the desired angular velocity to submit for the optimization and also set the linear part
    * to zero.
    * <p>
    * The {@code desiredAngularVelocity} has to define the desired angular velocity of the
    * end-effector with respect to the base. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be
    * attached to the end-effector. For instance, when controlling a foot, the {@code controlFrame}
    * should be located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredAngularVelocity the desired angular velocity of the end-effector with respect to
    *           the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredAngularVelocity} is not expressed in
    *            control frame.
    */
   public void setAngularVelocity(ReferenceFrame controlFrame, FrameVector3D desiredAngularVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularVelocity);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      this.desiredAngularVelocity.set(desiredAngularVelocity);
      desiredLinearVelocity.setToZero();
   }

   /**
    * Sets the desired linear velocity to submit for the optimization and also set the angular part
    * to zero.
    * <p>
    * The {@code desiredLinearVelocity} has to defined the desired linear velocity of the origin of
    * the {@code controlFrame}. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be
    * attached to the end-effector. For instance, when controlling a foot, the {@code controlFrame}
    * should be located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredLinearVelocity the desired linear velocity of the origin of the control frame
    *           with respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredLinearVelocity} is not expressed in
    *            control frame.
    */
   public void setLinearVelocity(ReferenceFrame controlFrame, FrameVector3D desiredLinearVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredLinearVelocity);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      this.desiredLinearVelocity.set(desiredLinearVelocity);
      desiredAngularVelocity.setToZero();
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom of the end-effector are to be controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.resetSelection();
   }

   /**
    * Convenience method that sets up the selection matrix such that only the linear part of this
    * command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
   }

   /**
    * Convenience method that sets up the selection matrix by disabling the angular part of this
    * command and applying the given selection matrix to the linear part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the
    * selection frame is equal to the control frame.
    * </p>
    * 
    * @param linearSelectionMatrix the selection matrix to apply to the linear part of this command.
    *           Not modified.
    */
   public void setSelectionMatrixForLinearControl(SelectionMatrix3D linearSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setLinearPart(linearSelectionMatrix);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the angular part of this
    * command will be considered in the optimization.
    */
   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.setToAngularSelectionOnly();
   }

   /**
    * Convenience method that sets up the selection matrix by disabling the linear part of this
    * command and applying the given selection matrix to the angular part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the
    * selection frame is equal to the control frame.
    * </p>
    * 
    * @param angularSelectionMatrix the selection matrix to apply to the angular part of this
    *           command. Not modified.
    */
   public void setSelectionMatrixForAngularControl(SelectionMatrix3D angularSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setAngularPart(angularSelectionMatrix);
   }

   /**
    * Convenience method that sets up the selection matrix such that only {@code x}, {@code z}, and
    * {@code pitch} are controlled.
    * <p>
    * This configuration is useful especially when dealing with planar robots evolving in the
    * XZ-plane.
    * </p>
    * 
    */
   public void setSelectionMatrixForPlanarControl()
   {
      selectionMatrix.clearSelection();
      selectionMatrix.selectAngularY(true);
      selectionMatrix.selectLinearX(true);
      selectionMatrix.selectLinearZ(true);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the {@code x} and
    * {@code z} components of the linear velocity are controlled.
    * <p>
    * This configuration is useful especially when dealing with planar robots evolving in the
    * XZ-plane.
    * </p>
    */
   public void setSelectionMatrixForPlanarLinearControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearY(false);
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
      this.selectionMatrix.set(selectionMatrix);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will
    * be treated as a hard equality constraint.
    * <p>
    * This is usually undesired as with improper commands setup as hard constraints the optimization
    * problem can simply be impossible to solve.
    * </p>
    */
   public void setAsHardEqualityConstraint()
   {
      this.constraintType = ConstraintType.EQUALITY;
      setWeight(HARD_CONSTRAINT);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will
    * be treated as a hard inequality constraint.
    * <p>
    * The optimization will then ensure that the spatial velocity of the end-effector is <b>less</b>
    * than or equal to the specified desired spatial velocity.
    * </p>
    */
   public void setAsLessOrEqualInequalityConstraint()
   {
      this.constraintType = ConstraintType.LEQ_INEQUALITY;
      setWeight(HARD_CONSTRAINT);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will
    * be treated as a hard inequality constraint.
    * <p>
    * The optimization will then ensure that the spatial velocity of the end-effector is
    * <b>greater</b> than or equal to the specified desired spatial velocity.
    * </p>
    */
   public void setAsGreaterOrEqualInequalityConstraint()
   {
      this.constraintType = ConstraintType.GEQ_INEQUALITY;
      setWeight(HARD_CONSTRAINT);
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
   public void setWeight(double weight)
   {
      this.constraintType = ConstraintType.OBJECTIVE;
      weightMatrix.setLinearWeights(weight, weight, weight);
      weightMatrix.setAngularWeights(weight, weight, weight);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weight to use for the angular part of this command. Not modified.
    * @param linear the weight to use for the linear part of this command. Not modified.
    */
   public void setWeight(double angular, double linear)
   {
      this.constraintType = ConstraintType.OBJECTIVE;
      weightMatrix.setLinearWeights(linear, linear, linear);
      weightMatrix.setAngularWeights(angular, angular, angular);
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
    *           velocity. Not modified.
    */
   public void setWeightMatrix(WeightMatrix6D weightMatrix)
   {
      this.constraintType = ConstraintType.OBJECTIVE;
      this.weightMatrix.set(weightMatrix);
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weights to use for the angular part of this command. Not modified.
    */
   public void setAngularWeights(Tuple3DReadOnly angular)
   {
      weightMatrix.setAngularWeights(angular.getX(), angular.getY(), angular.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param linear the weights to use for the angular part of this command. Not modified.
    */
   public void setLinearWeights(Tuple3DReadOnly linear)
   {
      this.constraintType = ConstraintType.OBJECTIVE;
      weightMatrix.setLinearWeights(linear.getX(), linear.getY(), linear.getZ());
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
   public void setWeights(Tuple3DReadOnly angular, Tuple3DReadOnly linear)
   {
      this.constraintType = ConstraintType.OBJECTIVE;
      weightMatrix.setLinearWeights(linear.getX(), linear.getY(), linear.getZ());
      weightMatrix.setAngularWeights(angular.getX(), angular.getY(), angular.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom to
    * zero.
    * <p>
    * By doing so, the angular part of this command will be ignored during the optimization.
    * </p>
    */
   public void setAngularWeightsToZero()
   {
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom
    * to zero.
    * <p>
    * By doing so, the linear part of this command will be ignored during the optimization. 
    * </p>
    */
   public void setLinearWeightsToZero()
   {
      weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
   }

   /**
    * Gets the 6-by-6 weight matrix expressed in the given {@code destinationFrame} to use with
    * this command.
    * 
    * @param destinationFrame the reference frame in which the weight matrix should be expressed
    *           in.
    * @param weightMatrixToPack the dense-matrix in which the weight matrix of this command is
    *           stored in. Modified.
    */
   public void getWeightMatrix(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      weightMatrix.getFullWeightMatrixInFrame(destinationFrame, weightMatrixToPack);
   }

   /**
    * Returns the weight Matrix used in this command:
    * 
    * @return the reference to the weight matrix to use with this command.
    */
   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   /**
    * Sets the weightMatrixToPack to the weight matrix used in this command:
    * @param weightMatrixToPack the weightMatrix To Pack. parameter is Modified
    */
   public void getWeightMatrix(WeightMatrix6D weightMatrixToPack)
   {
      weightMatrixToPack.set(weightMatrix);
   }

   /**
    * Packs the control frame and desired spatial velocity held in this command.
    * <p>
    * The first argument {@code controlFrameToPack} is required to properly express the
    * {@code desiredSpatialVelocityToPack}. Indeed the desired spatial velocity has to be
    * expressed in the control frame.
    * </p>
    * 
    * @param controlFrameToPack the frame of interest for controlling the end-effector. Modified.
    * @param desiredSpatialVelocityToPack the desired spatial velocity of the end-effector with
    *           respect to the base, expressed in the control frame. Modified.
    */
   public void getDesiredSpatialVelocity(PoseReferenceFrame controlFrameToPack, Twist desiredSpatialVelocityToPack)
   {
      getControlFrame(controlFrameToPack);
      desiredSpatialVelocityToPack.set(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(), controlFrameToPack, desiredLinearVelocity,
                                       desiredAngularVelocity);
   }

   /**
    * Packs the value of the desired spatial velocity of the end-effector with respect to the base,
    * expressed in the control frame.
    * <p>
    * The control frame can be obtained via {@link #getControlFrame(PoseReferenceFrame)}.
    * </p>
    * 
    * @param desiredSpatialVelocityToPack the 6-by-1 matrix in which the value of the desired
    *           spatial velocity is stored. The given matrix is reshaped to ensure proper size.
    *           Modified.
    */
   public void getDesiredSpatialVelocity(DenseMatrix64F desiredSpatialVelocityToPack)
   {
      desiredSpatialVelocityToPack.reshape(6, 1);
      desiredAngularVelocity.get(0, desiredSpatialVelocityToPack);
      desiredLinearVelocity.get(3, desiredSpatialVelocityToPack);
   }

   /**
    * Updates the given {@code PoseReferenceFrame} to match the control frame to use with this
    * command.
    * <p>
    * The control frame is assumed to be attached to the end-effector.
    * </p>
    * <p>
    * The spatial velocity that this command holds onto is expressed in the control frame.
    * </p>
    * 
    * @param controlFrameToPack the {@code PoseReferenceFrame} used to clone the control frame.
    *           Modified.
    */
   public void getControlFrame(PoseReferenceFrame controlFrameToPack)
   {
      controlFramePose.changeFrame(controlFrameToPack.getParent());
      controlFrameToPack.setPoseAndUpdate(controlFramePose);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
   }

   /**
    * Packs the pose of the control frame expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePoseToPack the pose of the control frame. Modified.
    */
   public void getControlFramePoseIncludingFrame(FramePose controlFramePoseToPack)
   {
      controlFramePoseToPack.setIncludingFrame(controlFramePose);
   }

   /**
    * Packs the position and orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param positionToPack the position of the {@code controlFrame}'s origin. Modified.
    * @param orientationToPack the orientation of the {@code controlFrame}. Modified.
    */
   public void getControlFramePoseIncludingFrame(FramePoint3D positionToPack, FrameQuaternion orientationToPack)
   {
      controlFramePose.getPositionIncludingFrame(positionToPack);
      controlFramePose.getOrientationIncludingFrame(orientationToPack);
   }

   /**
    * Gets the 6-by-6 selection matrix expressed in the given {@code destinationFrame} to use with
    * this command.
    * 
    * @param destinationFrame the reference frame in which the selection matrix should be expressed
    *           in.
    * @param selectionMatrixToPack the dense-matrix in which the selection matrix of this command is
    *           stored in. Modified.
    */
   public void getSelectionMatrix(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, selectionMatrixToPack);
   }

   /**
    * Packs the value of the selection matrix carried by this command into the given
    * {@code selectionMatrixToPack}.
    * 
    * @param selectionMatrixToPack the selection matrix to pack.
    */
   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   /**
    * Gets the reference to the base of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be used to control the end-effector.
    * </p>
    * 
    * @return the rigid-body located right before the first joint to be used for controlling the
    *         end-effector.
    */
   public RigidBody getBase()
   {
      return base;
   }

   /**
    * Gets the name of the base rigid-body.
    * 
    * @return the base's name.
    */
   public String getBaseName()
   {
      return baseName;
   }

   /**
    * Gets the reference to the end-effector of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be used to control the end-effector.
    * </p>
    * 
    * @return the rigid-body to be controlled.
    */
   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   /**
    * Gets the name of the end-effector rigid-body.
    * 
    * @return the end-effector's name.
    */
   public String getEndEffectorName()
   {
      return endEffectorName;
   }

   /**
    * Gets the reference to the primary base of this command.
    * <p>
    * This parameter is optional. If provided, it is used to improve singularity avoidance by
    * applying a privileged joint configuration to the kinematic chain going from
    * {@code primaryBase} to {@code endEffector}.
    * </p>
    * 
    * @return the rigid-body to use as the primary base. Optional.
    */
   public RigidBody getPrimaryBase()
   {
      return optionalPrimaryBase;
   }

   /**
    * Gets the name of the primary base rigid-body.
    * 
    * @return the primary base's name.
    */
   public String getPrimaryBaseName()
   {
      return optionalPrimaryBaseName;
   }

   /**
    * Gets whether or not to scale the weights on the joints below the {@code primaryBase}
    * when controlling the {@code endEffector}. A smaller scale (less than 1.0) means it will
    * use the joints in the kinematic chain between the {@code primaryBase} and the
    * {@code endEffector} more to control the {@code endEffector}, while a factor larger than
    * 1.0 makes it more likely to use the joints before the {@code primaryBase} (such as the
    * floating base) to control the {@code endEffector}.
    *
    * <p>
    *    This parameter is optional. If provided, it will scale the weights before the
    *    {@code primaryBase} by the factor defined in {@code secondaryTaskJointWeightScale}
    *    to control the {@code endEffector}.
    * </p>
    *
    * @return whether or not to scale the joints below the {@code primaryBase} (true) or not (false and default).
    */
   public boolean scaleSecondaryTaskJointWeight()
   {
      return scaleSecondaryTaskJointWeight;
   }

   /**
    * Gets the scaling factor for the weights on the joints below the {@code primaryBase}
    * when controlling the {@code endEffector}. A smaller scale (less than 1.0) means it will
    * use the joints in the kinematic chain between the {@code primaryBase} and the
    * {@code endEffector} more to control the {@code endEffector}, while a factor larger than
    * 1.0 makes it more likely to use the joints before the {@code primaryBase} (such as the
    * floating base) to control the {@code endEffector}.
    *
    * <p>
    *    This parameter is optional. If provided, it will be used to scale the weights before the
    *    {@code primaryBase} to control the {@code endEffector}.
    * </p>
    *
    * @return scale factor for the joints below the {@code primaryBase}.
    */
   public double getSecondaryTaskJointWeightScale()
   {
      return secondaryTaskJointWeightScale;
   }

   /**
    * Resets the secondary task joint weight scaling factor on the joints below the {@code primaryBase} to its
    * default value.
    */
   public void resetSecondaryTaskJointWeightScale()
   {
      secondaryTaskJointWeightScale = 1.0;
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.TASKSPACE;
   }

   /**
    * Get the type of spatial velocity constraint that should be imposed
    */
   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + ", endEffector = " + endEffector.getName() + ", linear = "
            + desiredLinearVelocity + ", angular = " + desiredAngularVelocity;
      return ret;
   }
}
