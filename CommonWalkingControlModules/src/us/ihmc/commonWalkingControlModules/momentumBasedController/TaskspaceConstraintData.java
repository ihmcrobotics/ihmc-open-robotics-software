package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

public class TaskspaceConstraintData
{
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);

   public void set(SpatialAccelerationVector spatialAcceleration)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.reshape(0, 1);
      this.selectionMatrix.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers, DenseMatrix64F selectionMatrix)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.setReshape(nullspaceMultipliers);
      this.selectionMatrix.setReshape(selectionMatrix);
   }

   public SpatialAccelerationVector getSpatialAcceleration()
   {
      return spatialAcceleration;
   }

   public DenseMatrix64F getNullspaceMultipliers()
   {
      return nullspaceMultipliers;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }
}
