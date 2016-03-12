package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FrameVector;
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

   public MomentumRateCommand(MomentumRateCommand momentumRateCommand)
   {
      this();
      set(momentumRateCommand);
   }

   @Override
   public void set(MomentumRateCommand other)
   {
      this.selectionMatrix.set(other.selectionMatrix);
      this.momentumRate.set(other.momentumRate);
   }

   public void set(SpatialForceVector momentumRateOfChange)
   {
      selectionMatrix.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      momentumRate.reshape(SpatialForceVector.SIZE, 1);
      momentumRateOfChange.getMatrix(momentumRate);
   }

   public void setLinearMomentumRateOfChange(FrameVector linearMomentumRateOfChange)
   {
      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);

      momentumRate.reshape(selectionMatrix.getNumCols(), 1);
      MatrixTools.setDenseMatrixFromTuple3d(momentumRate, linearMomentumRateOfChange.getVector(), 3, 0);
   }

   public void setEmpty()
   {
      selectionMatrix.reshape(0, SpatialForceVector.SIZE);
      momentumRate.reshape(0, 1);
   }

   public void setWeight(double weight)
   {
      setWeights(weight, weight);
   }

   public void setWeightLevel(InverseDynamicsCommandWeightLevels weightLevel)
   {
      setWeight(weightLevel.getWeightValue());
   }

   public void setWeights(double linear, double angular)
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, angular);
      for (int i = 3; i < SpatialAccelerationVector.SIZE; i++)
         weightVector.set(i, 0, linear);
   }

   public void setWeights(double linearX, double linearY, double linearZ, double angularX, double angularY, double angularZ)
   {
      weightVector.set(0, 0, angularX);
      weightVector.set(1, 0, angularY);
      weightVector.set(2, 0, angularZ);
      weightVector.set(3, 0, linearX);
      weightVector.set(4, 0, linearY);
      weightVector.set(5, 0, linearZ);
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public DenseMatrix64F getMomentumRate()
   {
      return momentumRate;
   }

   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
   }

   public void getWeightMatrix(DenseMatrix64F weightMatrix)
   {
      weightMatrix.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(weightMatrix);
      for (int i = 0; i < SpatialAccelerationVector.SIZE; i++)
         weightMatrix.set(i, i, weightVector.get(i, 0));
   }

   public void setMomentumRate(DenseMatrix64F momentumRate)
   {
      this.momentumRate.set(momentumRate);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   @Override
   public InverseDynamicsCommandType getCommandType()
   {
      return InverseDynamicsCommandType.MOMENTUM_RATE;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": selection matrix = " + selectionMatrix;
   }
}
