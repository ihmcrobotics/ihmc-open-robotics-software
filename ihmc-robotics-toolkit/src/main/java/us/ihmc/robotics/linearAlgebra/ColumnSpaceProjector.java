package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

/**
 * @author twan
 *         Date: 4/16/13
 */
public class ColumnSpaceProjector
{
   private final LinearSolver<DenseMatrix64F> solver;
   private final DenseMatrix64F APlus;
   private final DenseMatrix64F A;
   private final DenseMatrix64F tempVector;

   public ColumnSpaceProjector(LinearSolver<DenseMatrix64F> solver, int numRows, int numCols)
   {
      this.solver = solver;
      this.A = new DenseMatrix64F(numRows, numCols);
      this.APlus = new DenseMatrix64F(numCols, numRows);
      this.tempVector = new DenseMatrix64F(numCols, 1);
   }

   public void setA(DenseMatrix64F A)
   {
      this.A.set(A);
      solver.setA(A);
      solver.invert(APlus);
   }

   public void project(DenseMatrix64F b, DenseMatrix64F bPlus)
   {
      CommonOps.mult(APlus, b, tempVector);
      CommonOps.mult(A, tempVector, bPlus);
   }
}
