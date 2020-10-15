package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;

public interface JacobianSolver
{
   public abstract void setJacobian(DMatrixRMaj jacobianMatrix);

   public abstract void solve(DMatrixRMaj solutionToPack, DMatrixRMaj vector);
}
