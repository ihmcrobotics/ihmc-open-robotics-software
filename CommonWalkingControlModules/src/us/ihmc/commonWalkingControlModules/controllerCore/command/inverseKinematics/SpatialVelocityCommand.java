package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;

public class SpatialVelocityCommand implements InverseKinematicsCommand<SpatialVelocityCommand>
{
   private final Twist spatialVelocity = new Twist();
   private final DenseMatrix64F weightVector = new DenseMatrix64F(Twist.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(Twist.SIZE);

   private RigidBody base;
   private RigidBody endEffector;
   private RigidBody optionalPrimaryBase;

   private String baseName;
   private String endEffectorName;
   private String optionalPrimaryBaseName;

   public SpatialVelocityCommand()
   {
      setAsHardConstraint();
   }

   @Override
   public void set(SpatialVelocityCommand other)
   {
      setWeights(other.getWeightVector());

      spatialVelocity.set(other.getSpatialVelocity());
      selectionMatrix.set(other.getSelectionMatrix());
      base = other.getBase();
      endEffector = other.getEndEffector();
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;

      optionalPrimaryBase = other.optionalPrimaryBase;
      optionalPrimaryBaseName = other.optionalPrimaryBaseName;
   }

   /**
    * Copies all the fields of the given {@link SpatialAccelerationCommand} into this except for the
    * spatial acceleration.
    * 
    * @param command the command to copy the properties from. Not modified.
    */
   public void setProperties(SpatialAccelerationCommand command)
   {
      setWeights(command.getWeightVector());

      selectionMatrix.set(command.getSelectionMatrix());
      base = command.getBase();
      endEffector = command.getEndEffector();
      baseName = command.getBaseName();
      endEffectorName = command.getEndEffectorName();

      optionalPrimaryBase = command.getPrimaryBase();
      optionalPrimaryBaseName = command.getPrimaryBaseName();
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;

      baseName = base.getName();
      endEffectorName = endEffector.getName();
   }

   public void setPrimaryBase(RigidBody primaryBase)
   {
      optionalPrimaryBase = primaryBase;
      optionalPrimaryBaseName = primaryBase.getName();
   }

   public void setSpatialVelocity(Twist spatialVelocity)
   {
      this.spatialVelocity.set(spatialVelocity);
   }

   public void setAngularVelocity(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredAngularVelocity)
   {
      spatialVelocity.setToZero(bodyFrame, baseFrame, desiredAngularVelocity.getReferenceFrame());
      spatialVelocity.setAngularPart(desiredAngularVelocity.getVector());
   }

   public void setLinearVelocity(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector desiredLinearVelocity)
   {
      spatialVelocity.setToZero(bodyFrame, baseFrame, desiredLinearVelocity.getReferenceFrame());
      spatialVelocity.setLinearPart(desiredLinearVelocity.getVector());
      spatialVelocity.changeFrame(bodyFrame);
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);
   }

   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.reshape(3, Twist.SIZE);
      selectionMatrix.zero();
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
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

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > Twist.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != Twist.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   public void setAsHardConstraint()
   {
      setWeight(HARD_CONSTRAINT);
   }

   public void setWeight(double weight)
   {
      for (int i = 0; i < Twist.SIZE; i++)
         weightVector.set(i, 0, weight);
   }

   public void setWeight(double angular, double linear)
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, angular);
      for (int i = 3; i < Twist.SIZE; i++)
         weightVector.set(i, 0, linear);
   }

   public void setWeights(DenseMatrix64F weight)
   {
      for (int i = 0; i < Twist.SIZE; i++)
      {
         weightVector.set(i, 0, weight.get(i, 0));
      }
   }

   public void setAngularWeights(Vector3D angular)
   {
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
      weightVector.set(3, 0, linear.getX());
      weightVector.set(4, 0, linear.getY());
      weightVector.set(5, 0, linear.getZ());
   }

   public void setLinearWeightsToZero()
   {
      for (int i = 3; i < Twist.SIZE; i++)
         weightVector.set(i, 0, 0.0);
   }

   public boolean isHardConstraint()
   {
      for (int i = 0; i < Twist.SIZE; i++)
      {
         if (weightVector.get(i, 0) == HARD_CONSTRAINT)
            return true;
      }
      return false;
   }

   public void getWeightMatrix(DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < Twist.SIZE; i++)
         weightMatrixToPack.set(i, i, weightVector.get(i, 0));
   }

   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
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

   public RigidBody getPrimaryBase()
   {
      return optionalPrimaryBase;
   }

   public String getPrimaryBaseName()
   {
      return optionalPrimaryBaseName;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.TASKSPACE;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + ", endEffector = " + endEffector.getName() + ", linear = "
            + spatialVelocity.getLinearPartCopy() + ", angular = " + spatialVelocity.getAngularPartCopy();
      return ret;
   }
}
