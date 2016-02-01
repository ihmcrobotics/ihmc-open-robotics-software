package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.tools.exceptions.NoConvergenceException;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.functions.PDQuadraticMultivariateRealFunction;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;


public class JOptimizerConstrainedQPSolver extends ConstrainedQPSolver
{

   final LinearSolver<DenseMatrix64F> solver=new DampedLeastSquaresSolver(1);;
   /*
    *  minimizex (1/2)x'Qx+f'x  
    *  s.t.
    *  Ain x <= bin
    *  Aeq x = beq,  
    */
   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, 
            DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      //forming problem
      double[][] QArray = DenseMatrixToDoubleArray(Q), 
            AeqArray = DenseMatrixToDoubleArray(Aeq), 
            AinArray = DenseMatrixToDoubleArray(Ain); 
      double[] x0 = x.getData();

      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[Ain.numRows];
      for(int i=0;i<Ain.numRows;i++)
         inequalities[i] = new LinearMultivariateRealFunction(AinArray[i], bin.get(i, 0));
      
      OptimizationRequest or = new OptimizationRequest();
      if(inequalities.length>0)
        or.setFi(inequalities);
      or.setA(AeqArray);
      or.setB(beq.getData());
      or.setTolerance(1e-12);
      or.setToleranceFeas(1e-12);

      int returnCode; 
      JOptimizer opt = new JOptimizer();
      opt.setOptimizationRequest(or);
      try
      {
        if (initialize)
        {
          //finding a feasible starting point.
          LinearMultivariateRealFunction dummyLinearObj= new LinearMultivariateRealFunction(new double[] { 1., 2. }, 0);
          or.setF0(dummyLinearObj);
          returnCode=opt.optimize();
          x0 = opt.getOptimizationResponse().getSolution();
        }
        
        //qp
        or.setInitialPoint(x0);
        PDQuadraticMultivariateRealFunction obj = new PDQuadraticMultivariateRealFunction(QArray, f.getData(), 0);
        or.setF0(obj);
        returnCode=opt.optimize();
      }
      catch(Exception e)
      {
          System.out.println(e.getMessage());
          throw new NoConvergenceException();
      }
      double[] xopt = opt.getOptimizationResponse().getSolution();
      System.arraycopy(xopt, 0, x0, 0, xopt.length);
      return 0;
   }
   @Override
   public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin, DenseMatrix64F lb,
         DenseMatrix64F ub, DenseMatrix64F x, boolean initialize) throws NoConvergenceException
   {
      //TODO: automatically fold boxConstraints into inequality constraint
      throw new UnsupportedOperationException("JOptimizer does not support boxConstraints, please fold them into inequality constraints");
   }
   @Override
   public boolean supportBoxConstraints()
   {
      return false;
   }
   
}
