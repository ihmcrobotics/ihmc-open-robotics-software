package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;

public class FourBarKinematicLoopJacobianSolver 
{
   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private DenseMatrix64F jacobianMatrixToPack;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator)
   {
      this.fourBarCalculator = fourBarCalculator;
   }

   public void setJacobian()
   {
      double xdOut = 0;
      double ydOut = 0;
      double alphadOut = 0;
      
      jacobianMatrixToPack = new DenseMatrix64F(3, 1, true, new double[]{xdOut,ydOut, alphadOut});               
   }

   public void solve(DenseMatrix64F angularVelocityVector, DenseMatrix64F jacobianMatrix, DenseMatrix64F solutionToPack)
   {
      CommonOps.mult(jacobianMatrix, angularVelocityVector, solutionToPack);

   }
   
   public DenseMatrix64F getJacobianMatrix()
   {
      return jacobianMatrixToPack;
   }
}
