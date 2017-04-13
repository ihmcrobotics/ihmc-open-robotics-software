package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Momentum;

/**
 * {@link MomentumRateCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link MomentumRateCommand} is to notify the inverse dynamics optimization
 * module that the center of mass is to track a desired acceleration during the next control tick to
 * reach the desired rate of change of momentum set in this command.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class MomentumRateCommand implements InverseDynamicsCommand<MomentumRateCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /**
    * It defines the 6-components of the desired rate of change of momentum to achieve for the next
    * control.
    * <p>
    * More precisely, it represents the desired rate of change of momentum at the center of mass
    * while the data is expressed in world frame.
    * </p>
    */
   private final DenseMatrix64F momentumRateOfChange = new DenseMatrix64F(Momentum.SIZE, 1);
   /**
    * Weights on a per component basis to use in the optimization function. A higher weight means
    * higher priority of this task.
    */
   private final DenseMatrix64F weightVector = new DenseMatrix64F(Momentum.SIZE, 1);
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the momentum rate
    * that are to be controlled. A 6-by-6 identity matrix will request the control of all the 6
    * degrees of freedom.
    * <p>
    * The three first rows refer to the 3 rotational DoFs and the 3 last rows refer to the 3
    * translational DoFs of the end-effector. Removing a row to the selection matrix using for
    * instance {@link MatrixTools#removeRow(DenseMatrix64F, int)} is the quickest way to ignore a
    * specific DoF for the momentum rate.
    * </p>
    */
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public MomentumRateCommand()
   {
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(MomentumRateCommand other)
   {
      weightVector.set(other.weightVector);
      selectionMatrix.set(other.selectionMatrix);
      momentumRateOfChange.set(other.momentumRateOfChange);
   }

   /**
    * Sets the desired momentum rate to submit for the optimization to zero.
    */
   public void setMomentumRateToZero()
   {
      momentumRateOfChange.zero();
   }

   /**
    * Sets the desired rate of change of momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    * 
    * @param momentumRateOfChange the desired momentum rate at the center of mass expressed in world
    *           frame. Not modified.
    */
   public void setMomentumRate(DenseMatrix64F momentumRateOfChange)
   {
      this.momentumRateOfChange.set(momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    * 
    * @param angularMomentumRateOfChange the desired angular momentum rate at the center of mass
    *           expressed in world frame. Not modified.
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentumRateOfChange} or
    *            {@code desiredLineaerAcceleration} is not expressed in world frame.
    */
   public void setMomentumRate(FrameVector angularMomentumRateOfChange, FrameVector linearMomentumRateOfChange)
   {
      angularMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      angularMomentumRateOfChange.get(0, momentumRateOfChange);
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of angular momentum to submit for the optimization and sets
    * the linear part to zero.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    * 
    * @param angularMomentumRateOfChange the desired angular momentum rate at the center of mass
    *           expressed in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentumRateOfChange} is not
    *            expressed in world frame.
    */
   public void setAngularMomentumRate(FrameVector angularMomentumRateOfChange)
   {
      angularMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      angularMomentumRateOfChange.get(0, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of linear momentum to submit for the optimization and sets the
    * angular part to zero.
    * 
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentumRateOfChange} is not expressed
    *            in world frame.
    */
   public void setLinearMomentumRate(FrameVector linearMomentumRateOfChange)
   {
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   public void setLinearMomentumXYRate(FrameVector2d linearMomentumRateOfChange)
   {
      momentumRateOfChange.zero();
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom for the rate of change of momentum are to be
    * controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(Momentum.SIZE, Momentum.SIZE);
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
      selectionMatrix.reshape(3, Momentum.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the x and y components of
    * the linear part of this command will be considered in the optimization.
    *
    * <pre>
    *     / 0 0 0 1 0 0 \
    * S = |             |
    *     \ 0 0 0 0 1 0 /
    * </pre>
    */
   public void setSelectionMatrixForLinearXYControl()
   {
      selectionMatrix.reshape(2, Momentum.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
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
      selectionMatrix.reshape(3, Momentum.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   /**
    * Sets the selection matrix to be used for the next control tick.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the rate of change
    * of momentum that are to be controlled. A 6-by-6 identity matrix will request the control of
    * all the 6 degrees of freedom.
    * </p>
    * <p>
    * The three first rows refer to the 3 rotational DoFs and the 3 last rows refer to the 3
    * translational DoFs of the end-effector. Removing a row to the selection matrix using for
    * instance {@link MatrixTools#removeRow(DenseMatrix64F, int)} is the quickest way to ignore a
    * specific DoF for the rate of change of momentum.
    * </p>
    *
    * @param selectionMatrix the new selection matrix to be used. Not modified.
    * @throws RuntimeException if the selection matrix has a number of rows greater than 6 or has a
    *            number of columns different to 6.
    */
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > Momentum.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != Momentum.SIZE)
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
      for (int i = 0; i < Momentum.SIZE; i++)
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
      for (int i = 3; i < Momentum.SIZE; i++)
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
    * @param angularX the weight to use for the x-axis of the angular part of this command.
    * @param angularY the weight to use for the y-axis of the angular part of this command.
    * @param angularZ the weight to use for the z-axis of the angular part of this command.
    * @param linearX the weight to use for the x-axis of the linear part of this command.
    * @param linearY the weight to use for the y-axis of the linear part of this command.
    * @param linearZ the weight to use for the z-axis of the linear part of this command.
    */
   public void setWeights(double angularX, double angularY, double angularZ, double linearX, double linearY, double linearZ)
   {
      weightVector.set(0, 0, angularX);
      weightVector.set(1, 0, angularY);
      weightVector.set(2, 0, angularZ);
      weightVector.set(3, 0, linearX);
      weightVector.set(4, 0, linearY);
      weightVector.set(5, 0, linearZ);
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
      for (int i = 0; i < Momentum.SIZE; i++)
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
      for (int i = 3; i < Momentum.SIZE; i++)
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
      for (int i = 0; i < Momentum.SIZE; i++)
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
      weightMatrixToPack.reshape(Momentum.SIZE, Momentum.SIZE);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < Momentum.SIZE; i++)
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
    * Gets the reference to the selection matrix to use with this command.
    * 
    * @return the selection matrix.
    */
   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * Gets the reference to the desired rate of change of momentum carried by this command.
    * <p>
    * It represents the desired rate of change of momentum at the center of mass while the data is
    * expressed in world frame.
    * </p>
    * 
    * @return the desired rate of change of momentum.
    */
   public DenseMatrix64F getMomentumRate()
   {
      return momentumRateOfChange;
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
   public String toString()
   {
      String stringOfMomentumRate = EuclidCoreIOTools.getStringOf("(", ")", ",", momentumRateOfChange.getData());
      return getClass().getSimpleName() + ": momentum rate: " + stringOfMomentumRate + ", selection matrix = " + selectionMatrix;
   }
}
