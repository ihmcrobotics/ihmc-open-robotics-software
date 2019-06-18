package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link SpatialAccelerationCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link SpatialAccelerationCommand} is to notify the inverse dynamics
 * optimization module that the given end-effector is to track a desired acceleration during the
 * next control tick.
 * </p>
 * <p>
 * It is usually either the result of the {@link SpatialFeedbackController} or used when the
 * end-effector is in contact with the environment in which case the end-effector is usually
 * required to not accelerate so it can decently exert the wrench needed.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class SpatialAccelerationCommand implements InverseDynamicsCommand<SpatialAccelerationCommand>
{
   /** Defines the reference frame of interest. It is attached to the end-effector. */
   private final FramePose3D controlFramePose = new FramePose3D();

   /**
    * It defines the desired linear acceleration of the origin of the control frame, with respect to
    * the base. The vector is expressed in the control frame.
    */
   private final Vector3D desiredLinearAcceleration = new Vector3D();
   /**
    * It defines the desired angular acceleration of the control frame, with respect to the base. The
    * vector is expressed in the control frame.
    */
   private final Vector3D desiredAngularAcceleration = new Vector3D();

   /**
    * The Weight matrix describes the qp weights used for optimization. All weights are initially set
    * to NaN. If the weights are NaN the controller will use the default weights. A higher weight means
    * a higher priority of this task.
    */
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector that
    * are to be controlled. It is initialized such that the controller will by default control all the
    * end-effector DoFs.
    * <p>
    * If the selection frame is not set, it is assumed that the selection frame is equal to the control
    * frame.
    * </p>
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   /**
    * The base is the rigid-body located right before the first joint to be used for controlling the
    * end-effector.
    */
   private RigidBodyBasics base;
   /** The end-effector is the rigid-body to be controlled. */
   private RigidBodyBasics endEffector;
   /**
    * Intermediate base located between the {@code base} and {@code endEffector}. It can be null.
    */
   private RigidBodyBasics optionalPrimaryBase;

   /**
    * Flag to indicate whether or not to custom scale the weights below the intermediate base
    * {@code optionalPrimaryBase} to control against, as opposed to using the default weight in
    * {@link us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator#secondaryTaskJointsWeight}.
    */
   private boolean scaleSecondaryTaskJointWeight = false;

   /**
    * Scale factor to apply to the weights on the task below the {@code optionalPrimaryBase}. This
    * weight replaces the scale factor in
    * {@link us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator#secondaryTaskJointsWeight}.
    */
   private double secondaryTaskJointWeightScale = 1.0;

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public SpatialAccelerationCommand()
   {
      setAsHardConstraint();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(SpatialAccelerationCommand other)
   {
      controlFramePose.setIncludingFrame(other.controlFramePose);
      desiredLinearAcceleration.set(other.desiredLinearAcceleration);
      desiredAngularAcceleration.set(other.desiredAngularAcceleration);

      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
      base = other.getBase();
      endEffector = other.getEndEffector();

      optionalPrimaryBase = other.optionalPrimaryBase;
      scaleSecondaryTaskJointWeight = other.scaleSecondaryTaskJointWeight;
      secondaryTaskJointWeightScale = other.secondaryTaskJointWeightScale;
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
      this.base = base;
      this.endEffector = endEffector;
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
    * @param primaryBase the rigid-body to use as the primary base. Optional.
    */
   public void setPrimaryBase(RigidBodyBasics primaryBase)
   {
      optionalPrimaryBase = primaryBase;
   }

   /**
    * Indicates that we would like to custom scale the weights on the joints in the kinematic chain
    * below the {@code primaryBase} when controlling the {@code endEffector}.
    *
    * @param scaleSecondaryTaskJointWeight whether or not to use a custom scaling factor on the joints
    *           below the primary base. Optional.
    * @param secondaryTaskJointWeightScale custom scaling factor for the joints below the primary base.
    *           Optional.
    */
   public void setScaleSecondaryTaskJointWeight(boolean scaleSecondaryTaskJointWeight, double secondaryTaskJointWeightScale)
   {
      this.scaleSecondaryTaskJointWeight = scaleSecondaryTaskJointWeight;
      this.secondaryTaskJointWeightScale = secondaryTaskJointWeightScale;
   }

   /**
    * Sets the desired acceleration to submit for the optimization to zero.
    * <p>
    * This is useful when the end-effector is in contact with the environment. Its acceleration has to
    * be set to zero so it can exert the required wrench.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    */
   public void setSpatialAccelerationToZero(ReferenceFrame controlFrame)
   {
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredAngularAcceleration.setToZero();
      desiredLinearAcceleration.setToZero();
   }

   /**
    * Sets the desired acceleration to submit for the optimization.
    * <p>
    * It is important that the spatial acceleration describes the acceleration of the control frame
    * especially if only part of the angular acceleration is to be controlled.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredSpatialAcceleration the desired end-effector acceleration with respect to the base
    *           and expressed in the control frame.
    * @throws ReferenceFrameMismatchException if the {@code desiredSpatialAcceleration} is not setup as
    *            follows: {@code bodyFrame = endEffector.getBodyFixedFrame()},
    *            {@code baseFrame = base.getBodyFixedFrame()}, {@code expressedInFrame = controlFrame}.
    */
   public void setSpatialAcceleration(ReferenceFrame controlFrame, SpatialAccelerationReadOnly desiredSpatialAcceleration)
   {
      desiredSpatialAcceleration.getBodyFrame().checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      desiredSpatialAcceleration.getBaseFrame().checkReferenceFrameMatch(base.getBodyFixedFrame());
      desiredSpatialAcceleration.getReferenceFrame().checkReferenceFrameMatch(controlFrame);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredAngularAcceleration.set(desiredSpatialAcceleration.getAngularPart());
      desiredLinearAcceleration.set(desiredSpatialAcceleration.getLinearPart());
   }

   /**
    * Sets the desired acceleration to submit for the optimization.
    * <p>
    * The {@code desiredAngularAcceleration} has to define the desired angular acceleration of the
    * end-effector with respect to the base. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The {@code desiredLinearAcceleration} has to defined the desired linear acceleration of the
    * origin of the {@code controlFrame}. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredAngularAcceleration the desired angular acceleration of the end-effector with
    *           respect to the base. Not modified.
    * @param desiredLinearAcceleration the desired linear acceleration of the origin of the control
    *           frame with respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredAngularAcceleration} or
    *            {@code desiredLinearAcceleration} is not expressed in control frame.
    */
   public void setSpatialAcceleration(ReferenceFrame controlFrame, FrameVector3DReadOnly desiredAngularAcceleration,
                                      FrameVector3DReadOnly desiredLinearAcceleration)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularAcceleration);
      controlFrame.checkReferenceFrameMatch(desiredLinearAcceleration);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      this.desiredAngularAcceleration.set(desiredAngularAcceleration);
      this.desiredLinearAcceleration.set(desiredLinearAcceleration);
   }

   /**
    * Sets the desired angular acceleration to submit for the optimization and also set the linear part
    * to zero.
    * <p>
    * The {@code desiredAngularAcceleration} has to define the desired angular acceleration of the
    * end-effector with respect to the base. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    *
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredAngularAcceleration the desired angular acceleration of the end-effector with
    *           respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredAngularAcceleration} is not expressed in
    *            control frame.
    */
   public void setAngularAcceleration(ReferenceFrame controlFrame, FrameVector3DReadOnly desiredAngularAcceleration)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularAcceleration);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      this.desiredAngularAcceleration.set(desiredAngularAcceleration);
      desiredLinearAcceleration.setToZero();
   }

   /**
    * Sets the desired linear acceleration to submit for the optimization and also set the angular part
    * to zero.
    * <p>
    * The {@code desiredLinearAcceleration} has to defined the desired linear acceleration of the
    * origin of the {@code controlFrame}. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredLinearAcceleration the desired linear acceleration of the origin of the control
    *           frame with respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredLinearAcceleration} is not expressed in
    *            control frame.
    */
   public void setLinearAcceleration(ReferenceFrame controlFrame, FrameVector3DReadOnly desiredLinearAcceleration)
   {
      controlFrame.checkReferenceFrameMatch(desiredLinearAcceleration);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      this.desiredLinearAcceleration.set(desiredLinearAcceleration);
      desiredAngularAcceleration.setToZero();
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
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
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
    * Convenience method that sets up the selection matrix by disabling the linear part of this command
    * and applying the given selection matrix to the angular part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    * 
    * @param angularSelectionMatrix the selection matrix to apply to the angular part of this command.
    *           Not modified.
    */
   public void setSelectionMatrixForAngularControl(SelectionMatrix3D angularSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setAngularPart(angularSelectionMatrix);
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
      this.selectionMatrix.set(selectionMatrix);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will be
    * treated as a hard constraint.
    * <p>
    * This is usually undesired as with improper commands setup as hard constraints the optimization
    * problem can simply be impossible to solve.
    * </p>
    */
   public void setAsHardConstraint()
   {
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
    * @param angularX the weight to use for the x-axis of the angular part of this command.
    * @param angularY the weight to use for the y-axis of the angular part of this command.
    * @param angularZ the weight to use for the z-axis of the angular part of this command.
    * @param linearX the weight to use for the x-axis of the linear part of this command.
    * @param linearY the weight to use for the y-axis of the linear part of this command.
    * @param linearZ the weight to use for the z-axis of the linear part of this command.
    */
   public void setWeights(double angularX, double angularY, double angularZ, double linearX, double linearY, double linearZ)
   {
      weightMatrix.setLinearWeights(linearX, linearY, linearZ);
      weightMatrix.setAngularWeights(angularX, angularY, angularZ);
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
   public void setWeightMatrix(WeightMatrix6D weightMatrix)
   {
      this.weightMatrix.set(weightMatrix);
   }

   /**
    * Sets the linear weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param linearWeightMatrix weight matrix holding the linear weights to use for each component of
    *           the desired acceleration. Not modified.
    */
   public void setLinearPartOfWeightMatrix(WeightMatrix3D linearWeightMatrix)
   {
      this.weightMatrix.setLinearPart(linearWeightMatrix);
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
   public void setAngularPartOfWeightMatrix(WeightMatrix3D angularWeightMatrix)
   {
      this.weightMatrix.setAngularPart(angularWeightMatrix);
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
      weightMatrix.setLinearWeights(linear.getX(), linear.getY(), linear.getZ());
      weightMatrix.setAngularWeights(angular.getX(), angular.getY(), angular.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom to
    * zero.
    */
   public void setAngularWeightsToZero()
   {
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom to
    * zero.
    */
   public void setLinearWeightsToZero()
   {
      weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
   }

   /**
    * Finds if this command is to be considered as a hard constraint during the optimization.
    * <p>
    * This command is considered to be a hard constraint if at least one of the weights is equal to
    * {@link SolverWeightLevels#HARD_CONSTRAINT}.
    * </p>
    * 
    * @return {@code true} if this command should be considered as a hard constraint, {@code false} is
    *         it should be part of the optimization objective.
    */
   public boolean isHardConstraint()
   {
      return weightMatrix.containsHardConstraint();
   }

   /**
    * Gets the 6-by-6 weight matrix expressed in the given {@code destinationFrame} to use with this
    * command.
    * 
    * @param destinationFrame the reference frame in which the weight matrix should be expressed in.
    * @param weightMatrixToPack the dense-matrix in which the weight matrix of this command is stored
    *           in. Modified.
    */
   public void getWeightMatrix(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      weightMatrix.getFullWeightMatrixInFrame(destinationFrame, weightMatrixToPack);
   }

   /**
    * Returns the weight Matrix used in this command:
    * 
    * @return the reference to the weight matrix used in this command.
    */
   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   /**
    * Sets the weightMatrixToPack to the weight matrix used in this command:
    * 
    * @param weightMatrixToPack the weightMatrix To Pack. parameter is Modified
    */
   public void getWeightMatrix(WeightMatrix6D weightMatrixToPack)
   {
      weightMatrixToPack.set(weightMatrix);
   }

   public Vector3DBasics getDesiredLinearAcceleration()
   {
      return desiredLinearAcceleration;
   }

   public Vector3DBasics getDesiredAngularAcceleration()
   {
      return desiredAngularAcceleration;
   }

   /**
    * Packs the control frame and desired spatial acceleration held in this command.
    * <p>
    * The first argument {@code controlFrameToPack} is required to properly express the
    * {@code desiredSpatialAccelerationToPack}. Indeed the desired spatial acceleration has to be
    * expressed in the control frame.
    * </p>
    * 
    * @param controlFrameToPack the frame of interest for controlling the end-effector. Modified.
    * @param desiredSpatialAccelerationToPack the desired spatial acceleration of the end-effector with
    *           respect to the base, expressed in the control frame. Modified.
    */
   public void getDesiredSpatialAcceleration(PoseReferenceFrame controlFrameToPack, SpatialAcceleration desiredSpatialAccelerationToPack)
   {
      getControlFrame(controlFrameToPack);
      desiredSpatialAccelerationToPack.setIncludingFrame(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(), controlFrameToPack,
                                                         desiredAngularAcceleration, desiredLinearAcceleration);
   }

   /**
    * Packs the value of the desired spatial acceleration of the end-effector with respect to the base,
    * expressed in the control frame.
    * <p>
    * The control frame can be obtained via {@link #getControlFrame(PoseReferenceFrame)}.
    * </p>
    * 
    * @param desiredSpatialAccelerationToPack the 6-by-1 matrix in which the value of the desired
    *           spatial acceleration is stored. The given matrix is reshaped to ensure proper size.
    *           Modified.
    */
   public void getDesiredSpatialAcceleration(DenseMatrix64F desiredSpatialAccelerationToPack)
   {
      desiredSpatialAccelerationToPack.reshape(6, 1);
      desiredAngularAcceleration.get(0, desiredSpatialAccelerationToPack);
      desiredLinearAcceleration.get(3, desiredSpatialAccelerationToPack);
   }

   /**
    * Updates the given {@code PoseReferenceFrame} to match the control frame to use with this command.
    * <p>
    * The control frame is assumed to be attached to the end-effector.
    * </p>
    * <p>
    * The spatial acceleration that this command holds onto is expressed in the control frame.
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
   public void getControlFramePoseIncludingFrame(FramePose3DBasics controlFramePoseToPack)
   {
      controlFramePoseToPack.setIncludingFrame(controlFramePose);
   }

   /**
    * Gets a read only view of the pose of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    *
    * @return the pose of the control frame.
    */
   public FramePose3DBasics getControlFramePose()
   {
      return controlFramePose;
   }

   /**
    * Packs the position and orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param positionToPack the position of the {@code controlFrame}'s origin. Modified.
    * @param orientationToPack the orientation of the {@code controlFrame}. Modified.
    */
   public void getControlFramePoseIncludingFrame(FramePoint3DBasics positionToPack, FrameOrientation3DBasics orientationToPack)
   {
      positionToPack.setIncludingFrame(controlFramePose.getPosition());
      orientationToPack.setIncludingFrame(controlFramePose.getOrientation());
   }

   /**
    * Gets the internal reference to the selection matrix to use with this command.
    * 
    * @return the selection matrix.
    */
   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * Gets the 6-by-6 selection matrix expressed in the given {@code destinationFrame} to use with this
    * command.
    * 
    * @param destinationFrame the reference frame in which the selection matrix should be expressed in.
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
    * Packs the linear value of the selection matrix carried by this command into the given
    * {@code selectionMatrixToPack}.
    *
    * @param selectionMatrixToPack the linear selection matrix to pack.
    */
   public void getLinearSelectionMatrix(SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix.getLinearPart());
   }

   /**
    * Packs the angular value of the selection matrix carried by this command into the given
    * {@code selectionMatrixToPack}.
    *
    * @param selectionMatrixToPack the linear selection matrix to pack.
    */
   public void getAngularSelectionMatrix(SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix.getAngularPart());
   }

   /**
    * Gets the reference to the base of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be used to control the end-effector.
    * </p>
    * 
    * @return the rigid-body located right before the first joint to be used for controlling the
    *         end-effector.
    */
   public RigidBodyBasics getBase()
   {
      return base;
   }

   /**
    * Gets the reference to the end-effector of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be used to control the end-effector.
    * </p>
    * 
    * @return the rigid-body to be controlled.
    */
   public RigidBodyBasics getEndEffector()
   {
      return endEffector;
   }

   /**
    * Gets the reference to the primary base of this command.
    * <p>
    * This parameter is optional. If provided, it is used to improve singularity avoidance by applying
    * a privileged joint configuration to the kinematic chain going from {@code primaryBase} to
    * {@code endEffector}.
    * </p>
    * 
    * @return the rigid-body to use as the primary base. Optional.
    */
   public RigidBodyBasics getPrimaryBase()
   {
      return optionalPrimaryBase;
   }

   /**
    * Gets whether or not to scale the weights on the joints below the {@code primaryBase} when
    * controlling the {@code endEffector}. A smaller scale (less than 1.0) means it will use the joints
    * in the kinematic chain between the {@code primaryBase} and the {@code endEffector} more to
    * control the {@code endEffector}, while a factor larger than 1.0 makes it more likely to use the
    * joints before the {@code primaryBase} (such as the floating base) to control the
    * {@code endEffector}.
    *
    * <p>
    * This parameter is optional. If provided, it will scale the weights before the {@code primaryBase}
    * by the factor defined in {@code secondaryTaskJointWeightScale} to control the
    * {@code endEffector}.
    * </p>
    *
    * @return whether or not to scale the joints below the {@code primaryBase} (true) or not (false and
    *         default).
    */
   public boolean scaleSecondaryTaskJointWeight()
   {
      return scaleSecondaryTaskJointWeight;
   }

   /**
    * Gets the scaling factor for the weights on the joints below the {@code primaryBase} when
    * controlling the {@code endEffector}. A smaller scale (less than 1.0) means it will use the joints
    * in the kinematic chain between the {@code primaryBase} and the {@code endEffector} more to
    * control the {@code endEffector}, while a factor larger than 1.0 makes it more likely to use the
    * joints before the {@code primaryBase} (such as the floating base) to control the
    * {@code endEffector}.
    *
    * <p>
    * This parameter is optional. If provided, it will be used to scale the weights before the
    * {@code primaryBase} to control the {@code endEffector}.
    * </p>
    *
    * @return scale factor for the joints below the {@code primaryBase}.
    */
   public double getSecondaryTaskJointWeightScale()
   {
      return secondaryTaskJointWeightScale;
   }

   /**
    * Resets the secondary task joint weight scaling factor on the joints below the {@code primaryBase}
    * to its default value.
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

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof SpatialAccelerationCommand)
      {
         SpatialAccelerationCommand other = (SpatialAccelerationCommand) object;

         if (!controlFramePose.equals(other.controlFramePose))
            return false;
         if (!desiredLinearAcceleration.equals(other.desiredLinearAcceleration))
            return false;
         if (!desiredAngularAcceleration.equals(other.desiredAngularAcceleration))
            return false;
         if (!weightMatrix.equals(other.weightMatrix))
            return false;
         if (!selectionMatrix.equals(other.selectionMatrix))
            return false;
         if (base != other.base)
            return false;
         if (endEffector != other.endEffector)
            return false;
         if (optionalPrimaryBase != other.optionalPrimaryBase)
            return false;
         if (scaleSecondaryTaskJointWeight != other.scaleSecondaryTaskJointWeight)
            return false;
         if (secondaryTaskJointWeightScale != other.secondaryTaskJointWeightScale)
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
      return getClass().getSimpleName() + ": base = " + base + ", endEffector = " + endEffector + ", linear = " + desiredLinearAcceleration + ", angular = "
            + desiredAngularAcceleration;
   }
}
