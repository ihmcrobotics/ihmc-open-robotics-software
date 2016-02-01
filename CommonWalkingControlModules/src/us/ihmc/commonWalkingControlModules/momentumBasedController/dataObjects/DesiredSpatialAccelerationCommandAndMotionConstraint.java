package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class DesiredSpatialAccelerationCommandAndMotionConstraint
{
   private final DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand;
   private final MotionConstraintBlocks motionConstraintBlocks;
   private final DenseMatrix64F achievedSpatialAcceleration = new DenseMatrix64F(6, 1);
   
   public DesiredSpatialAccelerationCommandAndMotionConstraint(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, MotionConstraintBlocks motionConstraintBlocks)
   {
      this.desiredSpatialAccelerationCommand = desiredSpatialAccelerationCommand;
      this.motionConstraintBlocks = motionConstraintBlocks;
   }
   
   public void computeAchievedSpatialAcceleration(DenseMatrix64F achievedJointAccelerations)
   {
      achievedSpatialAcceleration.reshape(motionConstraintBlocks.getjFullBlock().getNumRows(), 1);
      CommonOps.mult(motionConstraintBlocks.getjFullBlock(), achievedJointAccelerations, achievedSpatialAcceleration);  
   }
   
   public DenseMatrix64F getAchievedSpatialAcceleration()
   {
      return achievedSpatialAcceleration;
   }
   
   public DenseMatrix64F getDesiredSpatialAcceleration()
   {
      return motionConstraintBlocks.getpBlock();
   }

   public DesiredSpatialAccelerationCommand getDesiredSpatialAccelerationCommand()
   {
      return desiredSpatialAccelerationCommand;
   }
   
}
