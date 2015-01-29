package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.QuadProgWrapper;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;


public class QuadProgSolver extends ConstrainedQPSolver
{
   /*
    * The problem is in the form:
    *
    * min 0.5 * x G x + g0 x
    * s.t.
    *   CE^T x + ce0 = 0
    *   CI^T x + ci0 >= 0
    */

   private static final boolean DEBUG = false;

   QuadProgWrapper qpWrapper;
   DenseMatrix64F negAin, negAeq;


   public QuadProgSolver(YoVariableRegistry parentRegistry)
   {
      this(1, 0, 0, parentRegistry);
   }

   public QuadProgSolver(int nvar, int neq, int nin, YoVariableRegistry parentRegistry)
   {
      qpWrapper = new QuadProgWrapper(nvar, neq, nin);
      allocateTempraryMatrixOnDemand(nvar, neq, nin);

   }

   private void allocateTempraryMatrixOnDemand(int nvar, int neq, int nin)
   {
      if ((negAin == null) || (negAin.numRows != nvar) || (negAin.numCols != nin))
      {
         printWarningIfDebug(getClass().getSimpleName() + ": Re-allocated negAin");
         negAin = new DenseMatrix64F(nvar, nin);
      }

      if ((negAeq == null) || (negAeq.numRows != nvar) || (negAeq.numCols != neq))
      {
         printWarningIfDebug(getClass().getSimpleName() + ": Re-allocated negAeq");
         negAeq = new DenseMatrix64F(nvar, neq);
      }
   }

   private void printWarningIfDebug(String warning)
   {
      if (DEBUG)
         System.err.println(warning);
   }

   @Override
   public boolean supportBoxConstraints()
   {
      return false;
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x,
                    boolean initialize)
           throws NoConvergenceException
   {
      return solve(Q, f, Aeq, beq, Ain, bin, null, null, x, initialize);
   }

   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb,
                    DenseMatrix64F ub, DenseMatrix64F x, boolean initialize)
           throws NoConvergenceException
   {
      allocateTempraryMatrixOnDemand(Aeq.numCols, Aeq.numRows, Ain.numRows);

      CommonOps.transpose(Aeq, this.negAeq);
      CommonOps.scale(-1, this.negAeq);
      CommonOps.transpose(Ain, this.negAin);
      CommonOps.scale(-1, this.negAin);

      return qpWrapper.solve(Q, f, negAeq, beq, negAin, bin, x, initialize);
   }
}
