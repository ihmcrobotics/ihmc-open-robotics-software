package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

public class MomentumRateOfChangeData
{
   private final DenseMatrix64F momentumSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F momentumMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public void setMomentumSubspace(DenseMatrix64F momentumSubspace)
   {
      this.momentumSubspace.setReshape(momentumSubspace);
   }

   public void setMomentumMultipliers(DenseMatrix64F momentumMultipliers)
   {
      this.momentumMultipliers.setReshape(momentumMultipliers);
   }

   public DenseMatrix64F getMomentumSubspace()
   {
      return momentumSubspace;
   }

   public DenseMatrix64F getMomentumMultipliers()
   {
      return momentumMultipliers;
   }
}
