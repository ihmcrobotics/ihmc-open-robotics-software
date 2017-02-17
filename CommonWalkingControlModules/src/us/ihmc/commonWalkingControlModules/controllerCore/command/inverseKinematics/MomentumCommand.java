package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class MomentumCommand implements InverseKinematicsCommand<MomentumCommand>
{
   private final DenseMatrix64F weightVector = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F momentum = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

   public MomentumCommand()
   {
   }

   public MomentumCommand(MomentumCommand momentumCommand)
   {
      this();
      set(momentumCommand);
   }

   @Override
   public void set(MomentumCommand other)
   {
      selectionMatrix.set(other.selectionMatrix);
      momentum.set(other.momentum);
   }

   public void set(SpatialForceVector momentum)
   {
      selectionMatrix.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      this.momentum.reshape(SpatialForceVector.SIZE, 1);
      momentum.getMatrix(this.momentum);
   }

   public void setLinearMomentum(FrameVector linearMomentum)
   {
      selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);
      selectionMatrix.set(2, 5, 1.0);

      momentum.reshape(selectionMatrix.getNumCols(), 1);
      linearMomentum.getVector().get(3, 0, momentum);
   }

   public void setLinearMomentumXY(FrameVector2d linearMomentum)
   {
      selectionMatrix.reshape(2, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 3, 1.0);
      selectionMatrix.set(1, 4, 1.0);

      momentum.reshape(selectionMatrix.getNumCols(), 1);
      linearMomentum.getVector().get(3, 0, momentum);
   }

   public void setEmpty()
   {
      selectionMatrix.reshape(0, SpatialForceVector.SIZE);
      momentum.reshape(0, 1);
   }

   public void setWeight(double weight)
   {
      setWeights(weight, weight);
   }

   public void setWeights(double linear, double angular)
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, angular);
      for (int i = 3; i < SpatialMotionVector.SIZE; i++)
         weightVector.set(i, 0, linear);
   }

   public void setWeights(double linearX, double linearY, double linearZ, double angularX, double angularY, double angularZ)
   {
      weightVector.set(0, 0, angularX);
      weightVector.set(1, 0, angularY);
      weightVector.set(2, 0, angularZ);
      weightVector.set(3, 0, linearX);
      weightVector.set(4, 0, linearY);
      weightVector.set(5, 0, linearZ);
   }

   public boolean isHardConstraint()
   {
      for (int i = 0; i < weightVector.getNumRows(); i++)
      {
         if (weightVector.get(i, 0) == HARD_CONSTRAINT)
            return true;
      }
      return false;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public DenseMatrix64F getMomentum()
   {
      return momentum;
   }

   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
   }

   public void getWeightMatrix(DenseMatrix64F weightMatrix)
   {
      weightMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(weightMatrix);
      for (int i = 0; i < SpatialMotionVector.SIZE; i++)
         weightMatrix.set(i, i, weightVector.get(i, 0));
   }

   public void setMomentum(DenseMatrix64F momentum)
   {
      momentum.set(momentum);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": selection matrix = " + selectionMatrix;
   }
}
