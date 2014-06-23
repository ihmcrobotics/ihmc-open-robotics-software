package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.exeptions.NoConvergenceException;

import com.sun.jna.Native;
import com.sun.jna.Platform;

public class OASESConstrainedQPSolver extends ConstrainedQPSolver
{
   /*
    *  minimizex (1/2)x'Qx+f'x  
    *  s.t.
    *  Ain x <= bin
    *  Aeq x = beq,  
    */
    public static native void initializeNative(int nvar, int ncon);
    public static native double getObjVal();
    public static native int solveNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int nWSR, double[] cputime, double[] x);

    static{
          if(System.getProperty("jna.library.path")==null)
            System.setProperty("jna.library.path",System.getProperty("java.library.path"));
          Native.register("OASESConstrainedQPSolver_rel");
    }
   
   int nWSR, nvar, ncon;

   public OASESConstrainedQPSolver()
   {
      this(1,1,10);
   }
   
   public OASESConstrainedQPSolver(int _nvar, int _ncon, int _nWSR)
   {
      nWSR = _nWSR;
      initialize(_nvar, _ncon);
   }
   private void initialize(int _nvar, int _ncon)
   {
      if(nvar != _nvar || ncon != _ncon)
        initializeNative(_nvar, _ncon);
   }

   @Override
   public void solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, 
         DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      
      if(Aeq.numCols!=Ain.numCols)
         throw new RuntimeException("in consistent constraints");
      
      initialize(Aeq.numCols, Aeq.numRows+Ain.numRows);
      
      double[] cputime = new double[1];

      DenseMatrix64F A = new DenseMatrix64F(Aeq.numRows+Ain.numRows, Aeq.numCols);
      CommonOps.insert(Aeq, A, 0, 0);
      CommonOps.insert(Ain, A, Aeq.numRows, 0);

      DenseMatrix64F lbA = new DenseMatrix64F(Aeq.numRows+Ain.numRows, 1);
      CommonOps.fill(lbA, Double.MIN_VALUE);
      CommonOps.insert(beq, lbA, 0, 0);

      DenseMatrix64F ubA = new DenseMatrix64F(Aeq.numRows+Ain.numRows, 1);
      CommonOps.insert(beq, A, 0, 0);
      CommonOps.insert(bin, A, beq.numRows, 0);

      //see C-code for retCode explaination
      int retCode = solveNative(Q.getData(), f.getData(), Aeq.getData(), null, null, lbA.getData(), ubA.getData(), nWSR, cputime, x.getData());
   }

}
