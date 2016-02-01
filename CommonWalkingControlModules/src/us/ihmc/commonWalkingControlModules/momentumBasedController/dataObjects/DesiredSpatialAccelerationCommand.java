package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class DesiredSpatialAccelerationCommand
{
   private boolean hasWeight;
   private double weight;
   private GeometricJacobian jacobian;
   private TaskspaceConstraintData taskspaceConstraintData;

   public DesiredSpatialAccelerationCommand()
   {
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY;
   }

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

   public void set(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      this.jacobian = jacobian;
      this.taskspaceConstraintData = taskspaceConstraintData;
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY; 
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
   
   public void setHasWeight(boolean hasWeight)
   {
      this.hasWeight = hasWeight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setJacobian(GeometricJacobian jacobian)
   {
      this.jacobian = jacobian;
   }

   public void setTaskspaceConstraintData(TaskspaceConstraintData taskspaceConstraintData)
   {
      this.taskspaceConstraintData = taskspaceConstraintData;
   }

   public String toString()
   {
      return "DesiredSpatialAccelerationCommand: GeometricJacobian = " + jacobian.getShortInfo() + ", taskspaceConstraintData = " + taskspaceConstraintData;
   }



}
