package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link MomentumCommand} is a command meant to be submitted to the {@link WholeBodyControllerCore}
 * via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link MomentumCommand} is to notify the inverse kinematics optimization
 * module to get the robot to achieve the desired momentum during the next control tick.
 * </p>
 * <p>
 * This command can notably be used to control the center of mass velocity by using the linear part
 * of the momentum.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class MomentumCommand implements InverseKinematicsCommand<MomentumCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /**
    * It defines the 6-components of the desired momentum to achieve for the next control.
    * <p>
    * More precisely, it represents the desired momentum at the center of mass while the data is
    * expressed in world frame.
    * </p>
    */
   private final DenseMatrix64F momentum = new DenseMatrix64F(Momentum.SIZE, 1);
   /**
    * Weights on a per component basis to use in the optimization function. A higher weight means
    * higher priority of this task.
    */
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the momentum that are
    * to be controlled. It is initialized such that the controller will by default control all the
    * DoFs.
    * <p>
    * If the selection frame is not set, it is assumed that the selection frame is equal to the
    * centroidal frame.
    * </p>
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public MomentumCommand()
   {
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
      weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(MomentumCommand other)
   {
      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
      momentum.set(other.momentum);
   }

   /**
    * Copies all the fields of the given {@link MomentumRateCommand} into this except for the desired
    * rate of change of momentum.
    * 
    * @param command the command to copy the properties from. Not modified.
    */
   public void setProperties(MomentumRateCommand command)
   {
      weightMatrix.set(command.getWeightMatrix());
      command.getSelectionMatrix(selectionMatrix);
   }

   /**
    * Sets the desired momentum to submit for the optimization to zero.
    */
   public void setMomentumToZero()
   {
      momentum.zero();
   }

   /**
    * Sets the desired momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired momentum at the center of mass while the data is expressed
    * in world frame.
    * </p>
    * 
    * @param momentum the desired momentum at the center of mass expressed in world frame. Not
    *           modified.
    */
   public void setMomentum(DenseMatrix64F momentum)
   {
      this.momentum.set(momentum);
   }

   /**
    * Sets the desired momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired momentum at the center of mass while the data is expressed
    * in world frame.
    * </p>
    * 
    * @param angularMomentum the desired angular momentum at the center of mass expressed in world
    *           frame. Not modified.
    * @param linearMomentum the desired linear momentum in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentum} or {@code linearMomentum} is
    *            not expressed in world frame.
    */
   public void setMomentum(FrameVector3DReadOnly angularMomentum, FrameVector3DReadOnly linearMomentum)
   {
      angularMomentum.checkReferenceFrameMatch(worldFrame);
      linearMomentum.checkReferenceFrameMatch(worldFrame);

      angularMomentum.get(0, momentum);
      linearMomentum.get(3, momentum);
   }

   /**
    * Sets the desired angular momentum to submit for the optimization and sets the linear part to
    * zero.
    * <p>
    * It is assumed to represent the desired momentum at the center of mass while the data is expressed
    * in world frame.
    * </p>
    * 
    * @param angularMomentum the desired angular momentum at the center of mass expressed in world
    *           frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentum} is not expressed in world
    *            frame.
    */
   public void setAngularMomentum(FrameVector3DReadOnly angularMomentum)
   {
      angularMomentum.checkReferenceFrameMatch(worldFrame);

      momentum.zero();
      angularMomentum.get(0, momentum);
   }

   /**
    * Sets the desired linear momentum to submit for the optimization and sets the angular part to
    * zero.
    * 
    * @param linearMomentum the desired linear momentum in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentum} is not expressed in world
    *            frame.
    */
   public void setLinearMomentum(FrameVector3DReadOnly linearMomentum)
   {
      linearMomentum.checkReferenceFrameMatch(worldFrame);

      momentum.zero();
      linearMomentum.get(3, momentum);
   }

   /**
    * Sets the desired linear momentum in the XY-plane to submit for the optimization and sets the
    * angular part and the z component of the linear part to zero.
    * 
    * @param linearMomentum the desired linear momentum in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentum} is not expressed in world
    *            frame.
    */
   public void setLinearMomentumXY(FrameVector2DReadOnly linearMomentum)
   {
      linearMomentum.checkReferenceFrameMatch(worldFrame);

      momentum.zero();
      linearMomentum.get(3, momentum);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom for the momentum are to be controlled.
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
    * Convenience method that sets up the selection matrix such that only the x and y components of the
    * linear part of this command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearXYControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearZ(false);
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
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the momentum that are
    * to be controlled. It is initialized such that the controller will by default control all the
    * DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the centroidal frame.
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
    * @param weightMatrix dense matrix holding the weights to use for each component of the desired
    *           acceleration. It is expected to be a 6-by-1 vector ordered as: {@code angularX},
    *           {@code angularY}, {@code angularZ}, {@code linearX}, {@code linearY}, {@code linearZ}.
    *           Not modified.
    */
   public void setWeights(WeightMatrix6D weightMatrix)
   {
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
      setLinearWeights(linear);
      setAngularWeights(angular);
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom to
    * zero.
    * <p>
    * By doing so, the angular part of this command will be ignored during the optimization. Note that
    * it is less expensive to call {@link #setSelectionMatrixForLinearControl()} as by doing so the
    * angular part will not be submitted to the optimization.
    * </p>
    */
   public void setAngularWeightsToZero()
   {
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom to
    * zero.
    * <p>
    * By doing so, the linear part of this command will be ignored during the optimization. Note that
    * it is less expensive to call {@link #setSelectionMatrixForAngularControl()} as by doing so the
    * linear part will not be submitted to the optimization.
    * </p>
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
    * The three first weights (w0, w1, w2) are the weights to use for the angular part of this command.
    * The three others (w3, w4, w5) are for the linear part.
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
      weightMatrixToPack.reshape(Momentum.SIZE, Momentum.SIZE);
      weightMatrix.getFullWeightMatrixInFrame(worldFrame, weightMatrixToPack);
   }

   /**
    * Returns the weight matrix used in this command:
    *
    * @return the reference to the weights to use with this command.
    */
   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
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

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * Gets the reference to the desired momentum carried by this command.
    * <p>
    * It represents the desired momentum at the center of mass while the data is expressed in world
    * frame.
    * </p>
    * 
    * @return the desired momentum.
    */
   public DenseMatrix64F getMomentum()
   {
      return momentum;
   }

   /**
    * Packs the value of the desired momentum into two frame vectors.
    * 
    * @param angularPartToPack frame vector to pack the desired angular momentum at the center of mass.
    *           Modified.
    * @param linearPartToPack frame vector to pack the desired linear momentum. Modified.
    */
   public void getMomentumRate(FrameVector3DBasics angularPartToPack, FrameVector3DBasics linearPartToPack)
   {
      angularPartToPack.setIncludingFrame(worldFrame, 0, momentum);
      linearPartToPack.setIncludingFrame(worldFrame, 3, momentum);
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MomentumCommand)
      {
         MomentumCommand other = (MomentumCommand) object;

         if (!MatrixFeatures.isEquals(momentum, other.momentum))
            return false;
         if (!weightMatrix.equals(other.weightMatrix))
            return false;
         if (!selectionMatrix.equals(other.selectionMatrix))
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
      String stringOfMomentum = EuclidCoreIOTools.getStringOf("(", ")", ",", momentum.getData());
      return getClass().getSimpleName() + ": momentum: " + stringOfMomentum + ", selection matrix = " + selectionMatrix;
   }
}
