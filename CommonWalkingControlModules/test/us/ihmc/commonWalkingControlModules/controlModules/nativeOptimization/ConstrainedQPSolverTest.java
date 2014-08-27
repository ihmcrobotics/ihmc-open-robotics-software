package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.utilities.exeptions.NoConvergenceException;

public class ConstrainedQPSolverTest
{


   @Test
   public void testSolveContrainedQP() throws NoConvergenceException
   {
      
      int nin=1, neq=1, nv=2;
      DenseMatrix64F Q = new DenseMatrix64F(nv, nv, true, 1,0,0,1);
      DenseMatrix64F f = new DenseMatrix64F(nv, 1, true, 1,0);
      DenseMatrix64F Aeq = new DenseMatrix64F(neq, nv, true, 1,1);
      DenseMatrix64F beq = new DenseMatrix64F(neq, 1, true, 0);
      DenseMatrix64F Ain = new DenseMatrix64F(nin, nv, true, 2,1);
      DenseMatrix64F bin = new DenseMatrix64F(nin, 1, true, 0);
      
      ConstrainedQPSolver[] optimizers = {new JOptimizerConstrainedQPSolver(), new OASESConstrainedQPSolver()};
      for(int i=0;i<optimizers.length;i++)
      {
        DenseMatrix64F x = new DenseMatrix64F(nv, 1, true, -1, 1);
        optimizers[i].solve(Q, f, Aeq, beq, Ain, bin, x, false);
        System.out.println("xopt="+x);
      }
   }
   
   
   public static void main(String[] arg) throws NoConvergenceException
   {
      new ConstrainedQPSolverTest().testSolveContrainedQP();
   }
   
   
}
