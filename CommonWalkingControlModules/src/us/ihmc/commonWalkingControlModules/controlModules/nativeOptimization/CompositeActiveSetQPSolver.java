package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPStandaloneSolver;
import us.ihmc.utilities.exceptions.NoConvergenceException;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class CompositeActiveSetQPSolver extends ConstrainedQPSolver {
   
   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   SimpleActiveSetQPStandaloneSolver solver = new SimpleActiveSetQPStandaloneSolver(10);
   ConstrainedQPSolver fullSolver = new QuadProgSolver(registry);
//   ConstrainedQPSolver fullSolver = new OASESConstrainedQPSolver(registry);
   boolean[] linearInequalityActiveSet;

   LongYoVariable fullSolverCount = new LongYoVariable("fullSolverCount", registry);
   LongYoVariable simpleSolverIterations= new LongYoVariable("simpleSolverIterations", registry);
   
   public CompositeActiveSetQPSolver(YoVariableRegistry parentRegistry)
   {
      //parentRegistry.addChild(registry);
   }
   
   
	@Override
	public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq,
			DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin,
			DenseMatrix64F x, boolean initialize) throws NoConvergenceException {
	   
	   //allocate on demand
	   if(linearInequalityActiveSet==null)
	      linearInequalityActiveSet = new boolean[Ain.numRows];
	   else if(linearInequalityActiveSet.length != Ain.numRows)
	   {
	      System.err.println("linearInequalitySize changes, cold start with empty set");
	      linearInequalityActiveSet = new boolean[Ain.numRows];
	   }
	   
	   if(initialize)
	      Arrays.fill(linearInequalityActiveSet, false);
	   int iter=solver.solve(Q, f, Aeq, beq, Ain, bin, linearInequalityActiveSet, x);
	   simpleSolverIterations.set(iter);
	   
	   if(iter<0)
	   {
	      fullSolverCount.increment();
	      Arrays.fill(linearInequalityActiveSet, false);
	      fullSolver.solve(Q, f, Aeq, beq, Ain, bin, x, initialize);
	   }
	   
	   return iter;
		
	}

	@Override
	public int solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq,
			DenseMatrix64F beq, DenseMatrix64F Ain, DenseMatrix64F bin,
			DenseMatrix64F lb, DenseMatrix64F ub, DenseMatrix64F x,
			boolean initialize) throws NoConvergenceException {
		throw new RuntimeException("Not Implemented");
	}

	@Override
	public boolean supportBoxConstraints() {
		return false;
	}

}
