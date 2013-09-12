package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class DesiredJointAccelerationCommandAndMotionConstraint
{
   private final DesiredJointAccelerationCommand desiredJointAccelerationCommand;
   private final MotionConstraintBlocks motionConstraintBlocks;
   private final DenseMatrix64F achievedJointAcceleration = new DenseMatrix64F(1, 1);
   
   public DesiredJointAccelerationCommandAndMotionConstraint(DesiredJointAccelerationCommand desiredJointAccelerationCommand, MotionConstraintBlocks motionConstraintBlocks)
   {
      this.desiredJointAccelerationCommand = desiredJointAccelerationCommand;
      this.motionConstraintBlocks = motionConstraintBlocks;
      
      if (motionConstraintBlocks == null) throw new RuntimeException("motionConstraintBlocks == null");
   }
   
   public void computeAchievedJointAcceleration(DenseMatrix64F achievedJointAccelerations)
   {
      this.achievedJointAcceleration.reshape(motionConstraintBlocks.getjFullBlock().getNumRows(), 1);
      CommonOps.mult(motionConstraintBlocks.getjFullBlock(), achievedJointAccelerations, achievedJointAcceleration);  
   }
   
   public DenseMatrix64F getAchievedJointAcceleration()
   {
      return achievedJointAcceleration;
   }

   public DesiredJointAccelerationCommand getDesiredJointAccelerationCommand()
   {
      return desiredJointAccelerationCommand;
   }
   
}


