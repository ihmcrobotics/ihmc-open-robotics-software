package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public interface MomentumControlModuleSolverListener
{
   public abstract void setCentroidalMomentumMatrix(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F momentumSubspace);

   public abstract void setPrimaryMotionConstraintJMatrix(DenseMatrix64F jPrimary);
   public abstract void setPrimaryMotionConstraintPVector(DenseMatrix64F pPrimary);
   public abstract void setPrimaryMotionConstraintCheck(DenseMatrix64F checkCopy);
   public abstract void setCheckJQEqualsZeroAfterSetConstraint(DenseMatrix64F denseMatrix64F);

   public abstract void setSecondaryMotionConstraintJMatrix(DenseMatrix64F jSecondary);
   public abstract void setSecondaryMotionConstraintPVector(DenseMatrix64F pSecondary);
   public abstract void setSecondaryMotionConstraintWeightMatrix(DenseMatrix64F weightMatrixSecondary);
   
   public abstract void setJointAccelerationSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations);
   public abstract void setOptimizationValue(double optimizationValue);
   public abstract void reviewSolution();
}
