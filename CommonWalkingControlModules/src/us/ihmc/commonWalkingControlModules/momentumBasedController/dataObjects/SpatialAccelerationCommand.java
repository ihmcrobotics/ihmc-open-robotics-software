package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class SpatialAccelerationCommand extends InverseDynamicsCommand
{
   private boolean hasWeight;
   private double weight;
   private GeometricJacobian jacobian;
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private RigidBody base;
   private RigidBody endEffector;

   public SpatialAccelerationCommand()
   {
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY;
   }

   public SpatialAccelerationCommand(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      this.jacobian = jacobian;
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY;

      spatialAcceleration.set(taskspaceConstraintData.getSpatialAcceleration());
      nullspaceMultipliers.set(taskspaceConstraintData.getNullspaceMultipliers());
      selectionMatrix.set(taskspaceConstraintData.getSelectionMatrix());
      base = taskspaceConstraintData.getBase();
      endEffector = taskspaceConstraintData.getEndEffector();
   }

   public SpatialAccelerationCommand(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight)
   {
      this.jacobian = jacobian;
      this.hasWeight = true;
      this.weight = weight;

      spatialAcceleration.set(taskspaceConstraintData.getSpatialAcceleration());
      nullspaceMultipliers.set(taskspaceConstraintData.getNullspaceMultipliers());
      selectionMatrix.set(taskspaceConstraintData.getSelectionMatrix());
      base = taskspaceConstraintData.getBase();
      endEffector = taskspaceConstraintData.getEndEffector();
   }

   public SpatialAccelerationCommand(SpatialAccelerationCommand spatialAccelerationCommand)
   {
      this.jacobian = spatialAccelerationCommand.jacobian;
      this.hasWeight = spatialAccelerationCommand.hasWeight;
      this.weight = spatialAccelerationCommand.weight;

      spatialAcceleration.set(spatialAccelerationCommand.getSpatialAcceleration());
      nullspaceMultipliers.set(spatialAccelerationCommand.getNullspaceMultipliers());
      selectionMatrix.set(spatialAccelerationCommand.getSelectionMatrix());
      base = spatialAccelerationCommand.getBase();
      endEffector = spatialAccelerationCommand.getEndEffector();
   }

   public void set(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      this.jacobian = jacobian;
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY;

      spatialAcceleration.set(taskspaceConstraintData.getSpatialAcceleration());
      nullspaceMultipliers.set(taskspaceConstraintData.getNullspaceMultipliers());
      selectionMatrix.set(taskspaceConstraintData.getSelectionMatrix());
      base = taskspaceConstraintData.getBase();
      endEffector = taskspaceConstraintData.getEndEffector();
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

   public RigidBody getBase()
   {
      return base;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
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
      spatialAcceleration.set(taskspaceConstraintData.getSpatialAcceleration());
      nullspaceMultipliers.set(taskspaceConstraintData.getNullspaceMultipliers());
      selectionMatrix.set(taskspaceConstraintData.getSelectionMatrix());
      base = taskspaceConstraintData.getBase();
      endEffector = taskspaceConstraintData.getEndEffector();
   }

   public String toString()
   {
      String ret = getClass().getSimpleName() + ": GeometricJacobian = " + jacobian.getShortInfo() + ", spatialAcceleration = " + spatialAcceleration;
      return ret;
   }
}
