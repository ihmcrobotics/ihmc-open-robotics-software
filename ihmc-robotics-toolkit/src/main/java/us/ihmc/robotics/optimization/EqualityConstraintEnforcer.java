package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

/**
 * See IHMCUtilities/technicalDocuments/Equality Constrained Least Squares.lyx
 * @author twan
 *         Date: 4/30/13
 */
public class EqualityConstraintEnforcer
{
   private final LinearSolverDense<DMatrixRMaj> solver;

   // results
   private final DMatrixRMaj q = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj jPlusp = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj x = new DMatrixRMaj(1, 1);

   // temp stuff
   private final DMatrixRMaj jPlus = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj aCopy = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj j = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj p = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj check = new DMatrixRMaj(1, 1);

   public EqualityConstraintEnforcer(LinearSolverDense<DMatrixRMaj> solver)
   {
      this.solver = solver;
   }

   public void setConstraint(DMatrixRMaj j, DMatrixRMaj p)
   {
      this.j.set(j);
      this.p.set(p);

      if (j.getNumRows() > 0)
      {
         // J^{+}
         jPlus.reshape(j.getNumCols(), j.getNumRows());
         solver.setA(j);
         solver.invert(jPlus);

         // J^{+} * p
         jPlusp.reshape(jPlus.getNumRows(), p.getNumCols());
         CommonOps_DDRM.mult(jPlus, p, jPlusp);

         // Q
         q.reshape(jPlus.getNumRows(), j.getNumCols());
         CommonOps_DDRM.setIdentity(q);
         CommonOps_DDRM.multAdd(-1.0, jPlus, j, q);
      }
      else
      {
         jPlus.reshape(j.getNumCols(), 0);
         q.reshape(j.getNumCols(), j.getNumCols());
         CommonOps_DDRM.setIdentity(q);
         jPlusp.reshape(j.getNumCols(), 1);
         jPlusp.zero();
      }
   }
   
   public DMatrixRMaj checkJQEqualsZeroAfterSetConstraint()
   {
      DMatrixRMaj checkJQEqualsZero = new DMatrixRMaj(this.j.getNumRows(), q.getNumCols());
      CommonOps_DDRM.mult(this.j, q, checkJQEqualsZero);
      return checkJQEqualsZero;
   }

   public void constrainEquation(DMatrixRMaj a, DMatrixRMaj b)
   {
      aCopy.set(a);
      CommonOps_DDRM.mult(aCopy, q, a);

      CommonOps_DDRM.multAdd(-1.0, aCopy, jPlusp, b);
   }

   public DMatrixRMaj constrainResult(DMatrixRMaj xBar)
   {
      x.reshape(xBar.getNumRows(), 1);
      CommonOps_DDRM.mult(q, xBar, x);
      CommonOps_DDRM.addEquals(x, jPlusp);

      return x;
   }

   public DMatrixRMaj getConstraintPseudoInverse()
   {
      return jPlus;
   }

   public boolean areConstraintsEnforcedSuccesfully(DMatrixRMaj x, DMatrixRMaj j, DMatrixRMaj p, double epsilon)
   {
      // This check is only valid if you use an 'exact' solver, not if you're using the damped least squares 'pseudoinverse'

      if (j.getNumRows() > 0)
      {
         check.reshape(p.getNumRows(), 1);
         CommonOps_DDRM.mult(j, x, check);
         CommonOps_DDRM.subtractEquals(check, p);
         return MatrixFeatures_DDRM.isConstantVal(check, 0.0, epsilon);
      }
      return true;
   }
}
