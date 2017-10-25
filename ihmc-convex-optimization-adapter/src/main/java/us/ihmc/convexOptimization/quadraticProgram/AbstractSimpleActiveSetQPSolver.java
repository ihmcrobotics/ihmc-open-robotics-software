package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;

public abstract class AbstractSimpleActiveSetQPSolver implements SimpleActiveSetQPSolverInterface
{

   protected final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(0, 0);
   protected double quadraticCostScalar;

   protected final DenseMatrix64F linearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F linearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   protected final DenseMatrix64F linearInequalityConstraintsCMatrixO = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F linearInequalityConstraintsDVectorO = new DenseMatrix64F(0, 0);

   protected final DenseMatrix64F variableLowerBounds = new DenseMatrix64F(0, 0);
   protected final DenseMatrix64F variableUpperBounds = new DenseMatrix64F(0, 0);
}
