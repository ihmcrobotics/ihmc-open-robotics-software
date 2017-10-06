package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

public interface JacobianSolver
{
   public abstract void setJacobian(DenseMatrix64F jacobianMatrix);

   public abstract void solve(DenseMatrix64F solutionToPack, DenseMatrix64F vector);
}
