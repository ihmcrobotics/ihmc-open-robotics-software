package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;

public class SpatialVelocityCommand implements InverseKinematicsCommand<SpatialVelocityCommand>
{
   private double weight;
   private final Twist spatialVelocity = new Twist();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(Twist.SIZE);

   private RigidBody base;
   private RigidBody endEffector;

   private String baseName;
   private String endEffectorName;

   public SpatialVelocityCommand()
   {
      removeWeight();
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      setBase(base);
      setEndEffector(endEffector);
   }

   public void setBase(RigidBody base)
   {
      this.base = base;
      baseName = base.getName();
   }

   public void setEndEffector(RigidBody endEffector)
   {
      this.endEffector = endEffector;
      endEffectorName = endEffector.getName();
   }

   public void set(Twist spatialVelocity)
   {
      this.spatialVelocity.set(spatialVelocity);
      setSelectionMatrixToIdentity();
   }

   public void set(Twist spatialVelocity, DenseMatrix64F selectionMatrix)
   {
      this.spatialVelocity.set(spatialVelocity);
      setSelectionMatrix(selectionMatrix);
   }

   public void setAngularVelocity(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularVelocity)
   {
      spatialVelocity.setToZero(bodyFrame, baseFrame, desiredAngularVelocity.getReferenceFrame());
      spatialVelocity.setAngularPart(desiredAngularVelocity.getVector());

      setSelectionMatrixForAngularControl();
   }

   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   public void setLinearVelocity(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearVelocity)
   {
      spatialVelocity.setToZero(bodyFrame, baseFrame, desiredLinearVelocity.getReferenceFrame());
      spatialVelocity.setLinearPart(desiredLinearVelocity.getVector());
      spatialVelocity.changeFrame(bodyFrame);

      setSelectionMatrixForLinearControl();
   }

   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   public void setSelectionMatrixForPlanarControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 1, 1.0);
      selectionMatrix.set(1, 3, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   public void setSelectionMatrixForPlanarLinearControl()
   {
      selectionMatrix.reshape(2, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 5, 1.0);
   }

   @Override
   public void set(SpatialVelocityCommand other)
   {
      weight = other.weight;

      spatialVelocity.set(other.getSpatialVelocity());
      selectionMatrix.set(other.getSelectionMatrix());
      base = other.getBase();
      endEffector = other.getEndEffector();
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;
   }

   private void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > Twist.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != Twist.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   public boolean isHardConstraint()
   {
      return weight == HARD_CONSTRAINT;
   }

   public double getWeight()
   {
      return weight;
   }

   public Twist getSpatialVelocity()
   {
      return spatialVelocity;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public RigidBody getBase()
   {
      return base;
   }

   public String getBaseName()
   {
      return baseName;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public String getEndEffectorName()
   {
      return endEffectorName;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void removeWeight()
   {
      setWeight(HARD_CONSTRAINT);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.TASKSPACE;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + "endEffector = " + endEffector.getName() + ", spatialVelocity = " + spatialVelocity;
      return ret;
   }
}
