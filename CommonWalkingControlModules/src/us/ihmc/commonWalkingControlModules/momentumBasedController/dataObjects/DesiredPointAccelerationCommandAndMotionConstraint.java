package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class DesiredPointAccelerationCommandAndMotionConstraint
{
   private final DesiredPointAccelerationCommand desiredPointAccelerationCommand;
   private final MotionConstraintBlocks motionConstraintBlocks;
   private final DenseMatrix64F achievedPointAcceleration = new DenseMatrix64F(6, 1);
   
   public DesiredPointAccelerationCommandAndMotionConstraint(DesiredPointAccelerationCommand desiredPointAccelerationCommand, MotionConstraintBlocks motionConstraintBlocks)
   {
      this.desiredPointAccelerationCommand = desiredPointAccelerationCommand;
      this.motionConstraintBlocks = motionConstraintBlocks;
   }
   
   public void computeAchievedSpatialAcceleration(DenseMatrix64F achievedJointAccelerations)
   {
      achievedPointAcceleration.reshape(motionConstraintBlocks.getjFullBlock().getNumRows(), 1);
      CommonOps.mult(motionConstraintBlocks.getjFullBlock(), achievedJointAccelerations, achievedPointAcceleration);  
   }
   
   public DenseMatrix64F getAchievedPointAcceleration()
   {
      return achievedPointAcceleration;
   }
   
}

