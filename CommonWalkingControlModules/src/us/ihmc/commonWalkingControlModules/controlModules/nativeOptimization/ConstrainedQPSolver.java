package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.tools.exceptions.NoConvergenceException;

public abstract class ConstrainedQPSolver
{

   /*
    *  minimizex (1/2)x'Qx+f'x  
    *  s.t.
    *  Ain x <= bin
    *  Aeq x = beq,  
    */
   public abstract int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, 
            DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x, boolean initialize) throws NoConvergenceException;

   public abstract int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, 
            DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb, DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException;
   
   public abstract boolean supportBoxConstraints();
   
   static double[][] DenseMatrixToDoubleArray(DenseMatrix64F Q)
   {
      double[][] Qarray = new double[Q.numRows][Q.numCols];
      for(int i=0;i<Q.numRows;i++)
         System.arraycopy(Q.getData(), Q.numCols*i, Qarray[i], 0, Q.numCols);
      return Qarray;
   }
   
   
}
