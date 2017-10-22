package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;

public class SimpleActiveSetQPSolver extends AbstractActiveSetQPSolver
{
   // Uses the algorithm and naming convention found in MIT Paper
   // "An efficiently solvable quadratic program for stabilizing dynamic locomotion"
   // by Scott Kuindersma, Frank Permenter, and Russ Tedrakenv.

   SimpleActiveSetQPStandaloneSolver solver = new SimpleActiveSetQPStandaloneSolver();
   DenseMatrix64F solutionVector = new DenseMatrix64F(0);

   @Override
   public double[] solve()
   {
      int iterations = solver.solve(quadraticCostGMatrix, quadraticCostFVector, linearEqualityConstraintA, linearEqualityConstraintB, linearInequalityConstraintA,
                          linearInequalityConstraintB, linearInequalityActiveSet, solutionVector);
      return solutionVector.getData();
   }
}
