package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

import com.sun.jna.Native;


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
    
    @Override
    public boolean supportBoxConstraints()
    {
       return true;
    }
    
    /**
     * 
     * min 0.5*x'Hx + g'x
     * 
     * st lbA <= Ax <= ubA
     *     lb <= x <= ub
     * 
     * matrices are row-major
     * 
     * @param nWSR - number of working set re-calculation
     * @param cputime - maximum cputime, null
     * @param x - initial and return variable to be optimized
     * @return returnCode from C-API
     */
    public static native int solveNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR, double[] cputime, double[] x);
    public static native int hotstartNative(double[] H, double[] g, double[] A, double[] lb, double[] ub, double[] lbA, double[] ubA, int[] nWSR, double[] cputime, double[] x);
    static{
          if(System.getProperty("jna.library.path")==null)
            System.setProperty("jna.library.path",System.getProperty("java.library.path"));
          try
          {
            Native.register("OASESConstrainedQPSolver_rel");
          }
          catch(UnsatisfiedLinkError e)
          {
            Native.register("OASESConstrainedQPSolver_msz");
          }
    }
   
   int nvar, ncon;
   double[] cputime = new double[1];
   int[] nWSR =new int[1];
   boolean coldStart;
   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   DoubleYoVariable maxCPUTime = new DoubleYoVariable("maxCPUTime", registry);
   DoubleYoVariable currentCPUTime = new DoubleYoVariable("currentCPUTime", registry);
   IntegerYoVariable maxWorkingSetChange = new IntegerYoVariable("maxWorkingSetchange", registry);
   IntegerYoVariable currentWorkingSetChange = new IntegerYoVariable("currentWorkingSetchange", registry);

   
   public OASESConstrainedQPSolver(int nvar, int ncon, YoVariableRegistry parentRegistry)
   {
      initialize(nvar, ncon);
      maxCPUTime.set(Double.POSITIVE_INFINITY);
      maxWorkingSetChange.set(10000);
      if(parentRegistry!=null)
        parentRegistry.addChild(this.registry);
   }

   public OASESConstrainedQPSolver(YoVariableRegistry parentRegistry)
   {
      this(1,1, parentRegistry);
   }

   private DenseMatrix64F A = null;
   private DenseMatrix64F lbA = null;
   private DenseMatrix64F ubA = null;

   private void initialize(int nvar, int ncon)
   {
      if(this.nvar != nvar || this.ncon != ncon)
      {
        initializeNative(nvar, ncon);
        this.nvar = nvar;
        this.ncon=ncon;
        A = new DenseMatrix64F(this.ncon, this.nvar);
        lbA = new DenseMatrix64F(ncon, 1);
        ubA = new DenseMatrix64F(ncon, 1);
        coldStart = true;
      }
   }
   

   @Override
   public void solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, 
         DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      solve(Q, f, Aeq, beq, Ain, bin, null, null, x, initialize);
   }
   /**
    *   min_x  x'Qx + f'x
    *   
    *   s.t. Ain x <= bin
    *        Aeq x = beq
    *        
    *   all vectors are column vectors.
    *   
    */
   public void solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, 
         DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb, DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      
      if(Aeq.numCols!=Ain.numCols || Aeq.numCols != x.numRows)
         throw new RuntimeException("inconsistent constraints");
      if(initialize)
         coldStart=true;

      initialize(Aeq.numCols, Aeq.numRows+Ain.numRows);
      

      CommonOps.insert(Aeq, A, 0, 0);
      CommonOps.insert(Ain, A, Aeq.numRows, 0);

      CommonOps.fill(lbA, Double.NEGATIVE_INFINITY);
      CommonOps.insert(beq, lbA, 0, 0);

      CommonOps.insert(beq, ubA, 0, 0);
      CommonOps.insert(bin, ubA, beq.numRows, 0);

      //see C-code for retCode explanation
      int retCode;
      double[] 
        lbData=(lb==null)?null:lb.getData(), 
        ubData=(ub==null)?null:ub.getData();
      
      
      cputime[0]=maxCPUTime.getDoubleValue();
      nWSR[0]=maxWorkingSetChange.getIntegerValue();
      if(coldStart)
      {
         retCode = solveNative(Q.getData(), f.getData(), A.getData(), lbData, ubData, lbA.getData(), ubA.getData(), nWSR, cputime, x.getData());
      }
      else
      {
         retCode = hotstartNative(Q.getData(), f.getData(), A.getData(), lbData, ubData, lbA.getData(), ubA.getData(), nWSR, cputime, x.getData());
      }
      currentCPUTime.set(cputime[0]);
      currentWorkingSetChange.set(nWSR[0]);

      coldStart=false;
   }
   
   public boolean isColdStart()
   {
      return coldStart;
   }
}
