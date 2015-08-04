package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public class MomentumRateOfChangeData
{
   private final ReferenceFrame centerOfMassFrame;
   private final DenseMatrix64F momentumSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F momentumMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public MomentumRateOfChangeData(ReferenceFrame centerOfMassFrame)
   {
      this.centerOfMassFrame = centerOfMassFrame;
   }

   public void set(SpatialForceVector momentumRateOfChange)
   {
      momentumRateOfChange.changeFrame(centerOfMassFrame);
      momentumSubspace.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
      CommonOps.setIdentity(momentumSubspace);
      momentumMultipliers.reshape(SpatialForceVector.SIZE, 1);
      momentumRateOfChange.packMatrix(momentumMultipliers);
   }

   public void setLinearMomentumRateOfChange(FrameVector linearMomentumRateOfChange)
   {
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      momentumSubspace.reshape(SpatialForceVector.SIZE, 3);
      momentumSubspace.set(3, 0, 1.0);
      momentumSubspace.set(4, 1, 1.0);
      momentumSubspace.set(5, 2, 1.0);

      momentumMultipliers.reshape(momentumSubspace.getNumCols(), 1);
      MatrixTools.setDenseMatrixFromTuple3d(momentumMultipliers, linearMomentumRateOfChange.getVector(), 0, 0);
   }

   public void setEmpty()
   {
      momentumSubspace.reshape(SpatialForceVector.SIZE, 0);
      momentumMultipliers.reshape(0, 1);
   }

   public DenseMatrix64F getMomentumSubspace()
   {
      return momentumSubspace;
   }

   public DenseMatrix64F getMomentumMultipliers()
   {
      return momentumMultipliers;
   }

   public void set(MomentumRateOfChangeData other)
   {
      this.momentumSubspace.set(other.momentumSubspace);
      this.momentumMultipliers.set(other.momentumMultipliers);
   }

   public void setMomentumMultipliers(DenseMatrix64F momentumMultipliers)
   {
      this.momentumMultipliers.set(momentumMultipliers);
   }

   public void setMomentumSubspace(DenseMatrix64F momentumSubspace)
   {
      this.momentumSubspace.set(momentumSubspace);
   }
}
