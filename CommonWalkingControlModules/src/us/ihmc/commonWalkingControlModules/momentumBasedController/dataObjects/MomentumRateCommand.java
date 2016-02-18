package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public class MomentumRateCommand extends InverseDynamicsCommand<MomentumRateCommand>
{
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F momentumRate = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public MomentumRateCommand()
   {
      super(InverseDynamicsCommandType.MOMENTUM_RATE);
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
      selectionMatrix.reshape(SpatialForceVector.SIZE, 3);
      selectionMatrix.set(3, 0, 1.0);
      selectionMatrix.set(4, 1, 1.0);
      selectionMatrix.set(5, 2, 1.0);

      momentumRate.reshape(selectionMatrix.getNumCols(), 1);
      MatrixTools.setDenseMatrixFromTuple3d(momentumRate, linearMomentumRateOfChange.getVector(), 0, 0);
   }

   public void setEmpty()
   {
      selectionMatrix.reshape(SpatialForceVector.SIZE, 0);
      momentumRate.reshape(0, 1);
   }

   public DenseMatrix64F getMomentumSubspace()
   {
      return selectionMatrix;
   }

   public DenseMatrix64F getMomentumMultipliers()
   {
      return momentumRate;
   }

   public void setMomentumMultipliers(DenseMatrix64F momentumMultipliers)
   {
      this.momentumRate.set(momentumMultipliers);
   }

   public void setMomentumSubspace(DenseMatrix64F momentumSubspace)
   {
      this.selectionMatrix.set(momentumSubspace);
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": MomentumSubspace = " + selectionMatrix;
   }
}
