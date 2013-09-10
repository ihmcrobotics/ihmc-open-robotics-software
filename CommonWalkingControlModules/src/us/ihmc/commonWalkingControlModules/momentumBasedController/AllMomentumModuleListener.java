package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.apache.commons.lang.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredMomentumModuleCommandListener;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionConstraintListener;

public class AllMomentumModuleListener implements MotionConstraintListener, DesiredMomentumModuleCommandListener
{

   public void jointAccelerationMotionConstraintWasAdded(DesiredJointAccelerationCommand desiredJointAccelerationCommand, int motionConstraintIndex,
         DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
    System.out.println("Joint Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
      
   }

   public void spatialAccelerationMotionConstraintWasAdded(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand, int motionConstraintIndex,
         DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      System.out.println("Spatial Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
      
   }

   public void pointAccelerationMotionConstraintWasAdded(DesiredPointAccelerationCommand desiredPointAccelerationCommand, int motionConstraintIndex,
         DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock, MutableDouble weightBlock)
   {
      System.out.println("Point Acceleration motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
      
   }
   
//   public void motionConstraintWasAdded(int motionConstraintIndex, DenseMatrix64F jFullBlock, DenseMatrix64F jBlockCompact, DenseMatrix64F pBlock,
//         MutableDouble weightBlock)
//   {
//      System.out.println("Motion constraint was added: index = " + motionConstraintIndex + ", jFullBlock = " + jFullBlock + ", jBlockCompact = " + jBlockCompact + ", pBlock = " + pBlock);
//
//   }

   public void desiredRateOfChangeOfMomentumWasSet(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      System.out.println("desiredRateOfChangeOfMomentum was set: " + desiredRateOfChangeOfMomentumCommand);
      
   }

   public void desiredJointAccelerationWasSet(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      System.out.println("desiredJointAcceleration was set: " + desiredJointAccelerationCommand);
      
   }

   public void desiredSpatialAccelerationWasSet(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      System.out.println("desiredSpatialAcceleration was set: " + desiredSpatialAccelerationCommand);
      
   }

   public void desiredPointAccelerationWasSet(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      System.out.println("desiredPointAcceleration was set: " + desiredPointAccelerationCommand);
      
   }



}
