package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

public class DesiredSpatialAccelerationCommand
{
   private final boolean hasWeight;
   private final double weight;
   private final GeometricJacobian jacobian;
   private final TaskspaceConstraintData taskspaceConstraintData;

   public DesiredSpatialAccelerationCommand(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      this.jacobian = jacobian;
      this.taskspaceConstraintData = taskspaceConstraintData;
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY;
   }

   public DesiredSpatialAccelerationCommand(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight)
   {
      this.jacobian = jacobian;
      this.taskspaceConstraintData = taskspaceConstraintData;
      this.hasWeight = true;
      this.weight = weight;
   }
   
   public DesiredSpatialAccelerationCommand(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      this.jacobian = desiredSpatialAccelerationCommand.jacobian;
      this.taskspaceConstraintData = desiredSpatialAccelerationCommand.taskspaceConstraintData;
      this.hasWeight = desiredSpatialAccelerationCommand.hasWeight;
      this.weight = desiredSpatialAccelerationCommand.weight;
   }

  
   public boolean getHasWeight()
   {
      return hasWeight;
   }

   public double getWeight()
   {
      return weight;
   }

   public GeometricJacobian getJacobian()
   {
      return jacobian;
   }

   public TaskspaceConstraintData getTaskspaceConstraintData()
   {
      return taskspaceConstraintData;
   }

   public String toString()
   {
      return "DesiredSpatialAccelerationCommand: GeometricJacobian = " + jacobian.getShortInfo() + ", taskspaceConstraintData = " + taskspaceConstraintData;
   }
   
   public void computeAchievedSpatialAcceleration(DenseMatrix64F achievedSpatialAcceleration)
   {
      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      InverseDynamicsJoint[] jointsInOrder = jacobian.getJointsInOrder();
      
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(jacobian.getNumberOfColumns(), 1);
      
      int index = 0;
      for (int i=0; i<jointsInOrder.length; i++)
      {
         InverseDynamicsJoint joint = jointsInOrder[i];
         
         jointsInOrder[i].packDesiredAccelerationMatrix(jointAccelerations, index);
         index = index + joint.getDegreesOfFreedom();
      }
      
      CommonOps.mult(jacobianMatrix, jointAccelerations, achievedSpatialAcceleration);
   }

}
