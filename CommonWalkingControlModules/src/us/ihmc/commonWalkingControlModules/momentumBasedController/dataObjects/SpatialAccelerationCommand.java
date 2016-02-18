package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class SpatialAccelerationCommand extends InverseDynamicsCommand<SpatialAccelerationCommand>
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
      super(InverseDynamicsCommandType.TASKSPACE_MOTION);
      removeWeight();
   }

   public SpatialAccelerationCommand(GeometricJacobian jacobian)
   {
      this();
      setJacobian(jacobian);
   }

   public SpatialAccelerationCommand(GeometricJacobian jacobian, double weight)
   {
      this(jacobian);
      setWeight(weight);
   }

   public SpatialAccelerationCommand(SpatialAccelerationCommand spatialAccelerationCommand)
   {
      this();
      set(spatialAccelerationCommand);
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
   }

   public void set(SpatialAccelerationVector spatialAcceleration)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      nullspaceMultipliers.reshape(0, 1);
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers, DenseMatrix64F selectionMatrix)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.set(nullspaceMultipliers);
      this.selectionMatrix.set(selectionMatrix);
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.set(nullspaceMultipliers);
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration,
         DenseMatrix64F nullspaceMultipliers)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      this.nullspaceMultipliers.set(nullspaceMultipliers);

      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      nullspaceMultipliers.reshape(0, 1);

      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);

      nullspaceMultipliers.reshape(0, 1);

      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration, DenseMatrix64F selectionMatrix)
   {
      spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);

      nullspaceMultipliers.reshape(0, 1);

      MathTools.checkIfInRange(selectionMatrix.numRows, 0, 3);

      this.selectionMatrix.set(selectionMatrix);
   }

   @Override
   public void set(SpatialAccelerationCommand other)
   {
      jacobian = other.jacobian;
      hasWeight = other.hasWeight;
      weight = other.weight;

      spatialAcceleration.set(other.getSpatialAcceleration());
      nullspaceMultipliers.set(other.getNullspaceMultipliers());
      selectionMatrix.set(other.getSelectionMatrix());
      base = other.getBase();
      endEffector = other.getEndEffector();
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
      hasWeight = weight != Double.POSITIVE_INFINITY;
   }

   private void removeWeight()
   {
      setWeight(Double.POSITIVE_INFINITY);
   }

   public void setJacobian(GeometricJacobian jacobian)
   {
      this.jacobian = jacobian;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": GeometricJacobian = " + jacobian.getShortInfo() + ", spatialAcceleration = " + spatialAcceleration;
      return ret;
   }
}
