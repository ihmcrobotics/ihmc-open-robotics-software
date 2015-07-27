package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;

public interface MotionConstraintListener
{
   public abstract void jointAccelerationMotionConstraintWasAdded(DesiredJointAccelerationCommand desiredJointAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock);
   public abstract void spatialAccelerationMotionConstraintWasAdded(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jacobianMatrix, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock);
   public abstract void pointAccelerationMotionConstraintWasAdded(DesiredPointAccelerationCommand desiredPointAccelerationCommand, int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock);
   public abstract void nullSpaceMultiplierForSpatialAccelerationMotionContraintWasAdded(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand,
         int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock);

// public abstract void motionConstraintWasAdded(int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock);

}
