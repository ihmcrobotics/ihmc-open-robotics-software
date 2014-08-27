package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.screwTheory.GeometricJacobian;

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

}
