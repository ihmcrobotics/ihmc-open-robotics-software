package us.ihmc.robotics.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;

/**
 * See IHMCUtilities/technicalDocuments/Equality Constrained Least Squares.lyx
 * @author twan
 *         Date: 4/30/13
 */
public class EqualityConstraintEnforcer
{
   private final LinearSolver<DenseMatrix64F> solver;

   // results
   private final DenseMatrix64F q = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F jPlusp = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F x = new DenseMatrix64F(1, 1);

   // temp stuff
   private final DenseMatrix64F jPlus = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F aCopy = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F j = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F p = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F check = new DenseMatrix64F(1, 1);

   public EqualityConstraintEnforcer(LinearSolver<DenseMatrix64F> solver)
   {
      this.solver = solver;
   }

   public void setConstraint(DenseMatrix64F j, DenseMatrix64F p)
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
         CommonOps.mult(jPlus, p, jPlusp);

         // Q
         q.reshape(jPlus.getNumRows(), j.getNumCols());
         CommonOps.setIdentity(q);
         CommonOps.multAdd(-1.0, jPlus, j, q);
      }
      else
      {
         jPlus.reshape(j.getNumCols(), 0);
         q.reshape(j.getNumCols(), j.getNumCols());
         CommonOps.setIdentity(q);
         jPlusp.reshape(j.getNumCols(), 1);
         jPlusp.zero();
      }
   }
   
   public DenseMatrix64F checkJQEqualsZeroAfterSetConstraint()
   {
      DenseMatrix64F checkJQEqualsZero = new DenseMatrix64F(this.j.getNumRows(), q.getNumCols());
      CommonOps.mult(this.j, q, checkJQEqualsZero);
      return checkJQEqualsZero;
   }

   public void constrainEquation(DenseMatrix64F a, DenseMatrix64F b)
   {
      aCopy.set(a);
      CommonOps.mult(aCopy, q, a);

      CommonOps.multAdd(-1.0, aCopy, jPlusp, b);
   }

   public DenseMatrix64F constrainResult(DenseMatrix64F xBar)
   {
      x.reshape(xBar.getNumRows(), 1);
      CommonOps.mult(q, xBar, x);
      CommonOps.addEquals(x, jPlusp);

      return x;
   }

   public DenseMatrix64F getConstraintPseudoInverse()
   {
      return jPlus;
   }

   public boolean areConstraintsEnforcedSuccesfully(DenseMatrix64F x, DenseMatrix64F j, DenseMatrix64F p, double epsilon)
   {
      // This check is only valid if you use an 'exact' solver, not if you're using the damped least squares 'pseudoinverse'

      if (j.getNumRows() > 0)
      {
         check.reshape(p.getNumRows(), 1);
         CommonOps.mult(j, x, check);
         CommonOps.subtractEquals(check, p);
         return MatrixFeatures.isConstantVal(check, 0.0, epsilon);
      }
      return true;
   }
}
