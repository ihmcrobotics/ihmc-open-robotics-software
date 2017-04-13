package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class MomentumRateCommand implements InverseDynamicsCommand<MomentumRateCommand>
{
   private final DenseMatrix64F weightVector = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F momentumRate = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public MomentumRateCommand()
   {
   }

   @Override
   public void set(MomentumRateCommand other)
   {
      selectionMatrix.set(other.selectionMatrix);
      momentumRate.set(other.momentumRate);
   }

   public void set(SpatialForceVector momentumRateOfChange)
   {
      setSelectionMatrixToIdentity();
      momentumRate.reshape(SpatialForceVector.SIZE, 1);
      momentumRateOfChange.getMatrix(momentumRate);
   }

   public void setLinearMomentumRateOfChange(FrameVector linearMomentumRateOfChange)
   {
      setSelectionMatrixForLinearControl();
      momentumRate.reshape(selectionMatrix.getNumCols(), 1);
      linearMomentumRateOfChange.getVector().get(3, 0, momentumRate);
   }

   public void setAngularMomentumRateOfChange(FrameVector angularMomentumRateOfChange)
   {
      setSelectionMatrixToIdentity();
      momentumRate.reshape(selectionMatrix.getNumCols(), 1);
      angularMomentumRateOfChange.getVector().get(0, 0, momentumRate);
   }

   public void setLinearMomentumXYRateOfChange(FrameVector2d linearMomentumRateOfChange)
   {
      setSelectionMatrixForLinearXYControl();
      momentumRate.reshape(selectionMatrix.getNumCols(), 1);
      linearMomentumRateOfChange.getVector().get(3, 0, momentumRate);
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
      selectionMatrix.reshape(3, SpatialForceVector.SIZE);
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
      selectionMatrix.reshape(2, SpatialMotionVector.SIZE);
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
      selectionMatrix.reshape(3, SpatialForceVector.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
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
      if (selectionMatrix.getNumRows() > SpatialForceVector.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != SpatialForceVector.SIZE)
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
      for (int i = 0; i < SpatialAccelerationVector.SIZE; i++)
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
      for (int i = 3; i < SpatialAccelerationVector.SIZE; i++)
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
      for (int i = 0; i < SpatialAccelerationVector.SIZE; i++)
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
      for (int i = 0; i < SpatialAccelerationVector.SIZE; i++)
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
      weightMatrixToPack.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < SpatialAccelerationVector.SIZE; i++)
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

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }
   
   public DenseMatrix64F getMomentumRate()
   {
      return momentumRate;
   }
   
   public void setMomentumRate(DenseMatrix64F momentumRate)
   {
      this.momentumRate.set(momentumRate);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": selection matrix = " + selectionMatrix;
   }
}
