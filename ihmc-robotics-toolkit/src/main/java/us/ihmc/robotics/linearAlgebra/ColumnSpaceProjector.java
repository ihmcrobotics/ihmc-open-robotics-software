package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

/**
 * @author twan
 *         Date: 4/16/13
 */
public class ColumnSpaceProjector
{
   private final LinearSolverDense<DMatrixRMaj> solver;
   private final DMatrixRMaj APlus;
   private final DMatrixRMaj A;
   private final DMatrixRMaj tempVector;

   public ColumnSpaceProjector(LinearSolverDense<DMatrixRMaj> solver, int numRows, int numCols)
   {
      this.solver = solver;
      this.A = new DMatrixRMaj(numRows, numCols);
      this.APlus = new DMatrixRMaj(numCols, numRows);
      this.tempVector = new DMatrixRMaj(numCols, 1);
   }

   public void setA(DMatrixRMaj A)
   {
      this.A.set(A);
      solver.setA(A);
      solver.invert(APlus);
   }

   public void project(DMatrixRMaj b, DMatrixRMaj bPlus)
   {
      CommonOps_DDRM.mult(APlus, b, tempVector);
      CommonOps_DDRM.mult(A, tempVector, bPlus);
   }
}
