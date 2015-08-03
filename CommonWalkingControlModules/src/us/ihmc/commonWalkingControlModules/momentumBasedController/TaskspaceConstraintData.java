package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class TaskspaceConstraintData
{
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private RigidBody base;
   private RigidBody endEffector;

   public void set(RigidBody base, RigidBody endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
   }
   
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
      this.nullspaceMultipliers.set(nullspaceMultipliers);
      this.selectionMatrix.set(selectionMatrix);
   }

   public void set(SpatialAccelerationVector spatialAcceleration, DenseMatrix64F nullspaceMultipliers)
   {
      this.spatialAcceleration.set(spatialAcceleration);
      this.nullspaceMultipliers.set(nullspaceMultipliers);
      this.selectionMatrix.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration,
         DenseMatrix64F nullspaceMultipliers)
   {
      this.spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      this.spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      this.nullspaceMultipliers.set(nullspaceMultipliers);
      
      this.selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      this.selectionMatrix.set(0, 0, 1.0);
      this.selectionMatrix.set(1, 1, 1.0);
      this.selectionMatrix.set(2, 2, 1.0);
   }

   public void setAngularAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularAcceleration)
   {
      this.spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredAngularAcceleration.getReferenceFrame());
      this.spatialAcceleration.setAngularPart(desiredAngularAcceleration.getVector());

      this.nullspaceMultipliers.reshape(0, 1);
      
      this.selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      this.selectionMatrix.set(0, 0, 1.0);
      this.selectionMatrix.set(1, 1, 1.0);
      this.selectionMatrix.set(2, 2, 1.0);
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration)
   {
      this.spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      this.spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);

      this.nullspaceMultipliers.reshape(0, 1);

      this.selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      this.selectionMatrix.set(0, 3, 1.0);
      this.selectionMatrix.set(1, 4, 1.0);
      this.selectionMatrix.set(2, 5, 1.0);
   }

   public void setLinearAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearAcceleration, DenseMatrix64F selectionMatrix)
   {
      this.spatialAcceleration.setToZero(bodyFrame, baseFrame, desiredLinearAcceleration.getReferenceFrame());
      this.spatialAcceleration.setLinearPart(desiredLinearAcceleration.getVector());
      spatialAcceleration.changeFrameNoRelativeMotion(bodyFrame);

      this.nullspaceMultipliers.reshape(0, 1);

      MathTools.checkIfInRange(selectionMatrix.numRows, 0, 3);

      this.selectionMatrix.set(selectionMatrix);
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

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append(getClass().getSimpleName());
      builder.append("spatialAcceleration: " + spatialAcceleration);
      builder.append("selectionMatrix:\n" + selectionMatrix);
      return builder.toString();
   }
}
