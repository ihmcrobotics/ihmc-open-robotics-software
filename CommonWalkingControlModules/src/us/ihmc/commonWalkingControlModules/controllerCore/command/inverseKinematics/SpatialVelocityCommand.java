package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;

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
    * Weights on a per component basis to use in the optimization function. A higher weight means
    * higher priority of this task.
    */
   private final DenseMatrix64F weightVector = new DenseMatrix64F(Twist.SIZE, 1);
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. A 6-by-6 identity matrix will request the control of all the 6
    * degrees of freedom.
    * <p>
    * The three first rows refer to the 3 rotational DoFs and the 3 last rows refer to the 3
    * translational DoFs of the end-effector. Removing a row to the selection matrix using for
    * instance {@link MatrixTools#removeRow(DenseMatrix64F, int)} is the quickest way to ignore a
    * specific DoF of the end-effector.
    * </p>
    */
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(Twist.SIZE);

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
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public SpatialVelocityCommand()
   {
      setAsHardConstraint();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(SpatialVelocityCommand other)
   {
      setWeights(other.getWeightVector());

      selectionMatrix.set(other.getSelectionMatrix());
      base = other.getBase();
      endEffector = other.getEndEffector();
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;

      optionalPrimaryBase = other.optionalPrimaryBase;
      optionalPrimaryBaseName = other.optionalPrimaryBaseName;

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
      setWeights(command.getWeightVector());

      selectionMatrix.set(command.getSelectionMatrix());
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
   public void setSpatialVelocity(ReferenceFrame controlFrame, FrameVector desiredAngularVelocity, FrameVector desiredLinearVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularVelocity);
      controlFrame.checkReferenceFrameMatch(desiredLinearVelocity);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredAngularVelocity.get(this.desiredAngularVelocity);
      desiredLinearVelocity.get(this.desiredLinearVelocity);
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
   public void setAngularVelocity(ReferenceFrame controlFrame, FrameVector desiredAngularVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularVelocity);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      desiredAngularVelocity.get(this.desiredAngularVelocity);
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
   public void setLinearVelocity(ReferenceFrame controlFrame, FrameVector desiredLinearVelocity)
   {
      controlFrame.checkReferenceFrameMatch(desiredLinearVelocity);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      desiredLinearVelocity.get(this.desiredLinearVelocity);
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
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the linear part of this
    * command will be considered in the optimization.
    * 
    * <pre>
    *     / 0 0 0 1 0 0 \
    * S = | 0 0 0 0 1 0 |
    *     \ 0 0 0 0 0 1 /
    * </pre>
    */
   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the angular part of this
    * command will be considered in the optimization.
    * 
    * <pre>
    *     / 1 0 0 0 0 0 \
    * S = | 0 1 0 0 0 0 |
    *     \ 0 0 1 0 0 0 /
    * </pre>
    */
   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   /**
    * Convenience method that sets up the selection matrix such that only {@code x}, {@code z}, and
    * {@code pitch} are controlled.
    * <p>
    * This configuration is useful especially when dealing with planar robots evolving in the
    * XZ-plane.
    * </p>
    * 
    * <pre>
    *     / 0 1 0 0 0 0 \
    * S = | 0 0 0 1 0 0 |
    *     \ 0 0 0 0 0 1 /
    * </pre>
    * 
    */
   public void setSelectionMatrixForPlanarControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 1, 1.0);
      selectionMatrix.set(1, 3, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the {@code x} and
    * {@code z} components of the linear velocity are controlled.
    * <p>
    * This configuration is useful especially when dealing with planar robots evolving in the
    * XZ-plane.
    * </p>
    * 
    * <pre>
    *     / 0 0 0 1 0 0 \
    * S = |             |
    *     \ 0 0 0 0 0 1 /
    * </pre>
    * 
    */
   public void setSelectionMatrixForPlanarLinearControl()
   {
      selectionMatrix.reshape(2, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 5, 1.0);
   }

   /**
    * Sets the selection matrix to be used for the next control tick.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. A 6-by-6 identity matrix will request the control of all the 6
    * degrees of freedom.
    * </p>
    * <p>
    * The three first rows refer to the 3 rotational DoFs and the 3 last rows refer to the 3
    * translational DoFs of the end-effector. Removing a row to the selection matrix using for
    * instance {@link MatrixTools#removeRow(DenseMatrix64F, int)} is the quickest way to ignore a
    * specific DoF of the end-effector.
    * </p>
    * 
    * @param selectionMatrix the new selection matrix to be used. Not modified.
    * @throws RuntimeException if the selection matrix has a number of rows greater than 6 or has a
    *            number of columns different to 6.
    */
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > Twist.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != Twist.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will
    * be treated as a hard constraint.
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
      for (int i = 0; i < Twist.SIZE; i++)
         weightVector.set(i, 0, weight);
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
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, angular);
      for (int i = 3; i < Twist.SIZE; i++)
         weightVector.set(i, 0, linear);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param weight dense matrix holding the weights to use for each component of the desired
    *           acceleration. It is expected to be a 6-by-1 vector ordered as: {@code angularX},
    *           {@code angularY}, {@code angularZ}, {@code linearX}, {@code linearY},
    *           {@code linearZ}. Not modified.
    */
   public void setWeights(DenseMatrix64F weight)
   {
      for (int i = 0; i < Twist.SIZE; i++)
      {
         weightVector.set(i, 0, weight.get(i, 0));
      }
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
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
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
      weightVector.set(3, 0, linear.getX());
      weightVector.set(4, 0, linear.getY());
      weightVector.set(5, 0, linear.getZ());
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
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
      weightVector.set(3, 0, linear.getX());
      weightVector.set(4, 0, linear.getY());
      weightVector.set(5, 0, linear.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom to
    * zero.
    * <p>
    * By doing so, the angular part of this command will be ignored during the optimization. Note
    * that it is less expensive to call {@link #setSelectionMatrixForLinearControl()} as by doing so
    * the angular part will not be submitted to the optimization.
    * </p>
    */
   public void setAngularWeightsToZero()
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, 0.0);
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom
    * to zero.
    * <p>
    * By doing so, the linear part of this command will be ignored during the optimization. Note
    * that it is less expensive to call {@link #setSelectionMatrixForAngularControl()} as by doing
    * so the linear part will not be submitted to the optimization.
    * </p>
    */
   public void setLinearWeightsToZero()
   {
      for (int i = 3; i < SpatialAccelerationVector.SIZE; i++)
         weightVector.set(i, 0, 0.0);
   }

   /**
    * Finds if this command is to be considered as a hard constraint during the optimization.
    * <p>
    * This command is considered to be a hard constraint if at least one of the weights is equal to
    * {@link SolverWeightLevels#HARD_CONSTRAINT}.
    * </p>
    * 
    * @return {@code true} if this command should be considered as a hard constraint, {@code false}
    *         is it should be part of the optimization objective.
    */
   public boolean isHardConstraint()
   {
      for (int i = 0; i < Twist.SIZE; i++)
      {
         if (weightVector.get(i, 0) == HARD_CONSTRAINT)
            return true;
      }
      return false;
   }

   /**
    * Packs the weights to use for this command in {@code weightMatrixToPack} as follows:
    * 
    * <pre>
    *     / w0 0  0  0  0  0  \
    *     | 0  w1 0  0  0  0  |
    * W = | 0  0  w2 0  0  0  |
    *     | 0  0  0  w3 0  0  |
    *     | 0  0  0  0  w4 0  |
    *     \ 0  0  0  0  0  w5 /
    * </pre>
    * <p>
    * The three first weights (w0, w1, w2) are the weights to use for the angular part of this
    * command. The three others (w3, w4, w5) are for the linear part.
    * </p>
    * <p>
    * The packed matrix is a 6-by-6 matrix independently from the selection matrix.
    * </p>
    * 
    * @param weightMatrixToPack the weight matrix to use in the optimization. The given matrix is
    *           reshaped to ensure proper size. Modified.
    */
   public void getWeightMatrix(DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < Twist.SIZE; i++)
         weightMatrixToPack.set(i, i, weightVector.get(i, 0));
   }

   /**
    * Returns the reference to the 6-by-1 weight vector to use with this command:
    * 
    * <pre>
    *     / w0 \
    *     | w1 |
    * W = | w2 |
    *     | w3 |
    *     | w4 |
    *     \ w5 /
    * </pre>
    * <p>
    * The three first weights (w0, w1, w2) are the weights to use for the angular part of this
    * command. The three others (w3, w4, w5) are for the linear part.
    * </p>
    * 
    * @return the reference to the weights to use with this command.
    */
   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
   }

   /**
    * Packs the control frame and desired spatial velocity held in this command.
    * <p>
    * The first argument {@code controlFrameToPack} is required to properly express the
    * {@code desiredSpatialVelocityToPack}. Indeed the desired spatial acceleration has to be
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
   public void getControlFramePoseIncludingFrame(FramePoint positionToPack, FrameOrientation orientationToPack)
   {
      controlFramePose.getPositionIncludingFrame(positionToPack);
      controlFramePose.getOrientationIncludingFrame(orientationToPack);
   }

   /**
    * Gets the reference to the selection matrix to use with this command.
    * 
    * @return the selection matrix.
    */
   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
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
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + ", endEffector = " + endEffector.getName() + ", linear = "
            + desiredLinearVelocity + ", angular = " + desiredAngularVelocity;
      return ret;
   }
}
