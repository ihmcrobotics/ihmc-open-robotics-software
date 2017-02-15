package us.ihmc.convexOptimization.jOptimizer;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.functions.PSDQuadraticMultivariateRealFunction;
import com.joptimizer.functions.SOCPLogarithmicBarrier;
import com.joptimizer.functions.SOCPLogarithmicBarrier.SOCPConstraintParameters;
import com.joptimizer.optimizers.BarrierMethod;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;
import com.joptimizer.optimizers.OptimizationResponse;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.EXCLUDE}) // Revisit JOptimzer some day and see if they ever got their act in gear...
public class SimpleJOptimizerTest
{
   private static final boolean VERBOSE = true;

   /**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout = 300000)
   public void testReallyReallySimpleOptimizationProblem() throws Exception
   {
      // Minimize x subject to x = 2

      double[] minimizeF = new double[] {1.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // equalities
      double[][] equalityAMatrix = new double[][]{{1.0}};
      double[] equalityBVector = new double[]{2.0};
      
      
      double[] solution = solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector);

      if (VERBOSE) System.out.println("testReallyReallySimpleOptimizationProblem: solution = (" + solution[0] + ")");
      
      assertEquals(2.0, solution[0], 1e-5);
   }
   

   /**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testASimpleRedundantEqualityCase() throws Exception
   {
      // Minimize x subject to x = 2 and x = 2;
      double[] minimizeF = new double[] {1.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // equalities
      double[][] equalityAMatrix = new double[][]{{1.0}, {1.0}};
      double[] equalityBVector = new double[]{2.0, 2.0};
      
      double[] solution = solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector);

      if (VERBOSE) System.out.println("testReallyReallySimpleOptimizationProblem: solution = (" + solution[0] + ")");
      
      assertEquals(2.0, solution[0], 1e-5);
   }
   

   /**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testAnotherReallySimpleOptimizationProblem() throws Exception
   {
      // Minimize x subject to -x <= -2  (x >= 2);
      double[] minimizeF = new double[] {1.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[1];
      inequalities[0] = new LinearMultivariateRealFunction(new double[] {-1.0}, 2.0);    // -1.0 x + 2.0 <= 0

      double[] solution = solveOptimizationProblem(objectiveFunction, inequalities);

      if (VERBOSE) System.out.println("testAnotherReallySimpleOptimizationProblem: solution = (" + solution[0] + ")");

      assertEquals(2.0, solution[0], 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 3000)
   public void testReallySimpleOptimizationProblem() throws Exception
   {
      // Minimize -x subject to x <= 5

      double[] minimizeF = new double[] {-1.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // inequalities
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[1];
      inequalities[0] = new LinearMultivariateRealFunction(new double[] {1.0}, -5.0);    // 1.0 x - 5.0 <= 0

      double[] solution = solveOptimizationProblem(objectiveFunction, inequalities);

      if (VERBOSE) System.out.println("testReallySimpleOptimizationProblem: solution = (" + solution[0] + ")");
      
      assertEquals(5.0, solution[0], 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 3000)
   public void testLinearCostQuadraticInequalityOptimizationProblem() throws Exception
   {
      // Minimize -x-y subject to x^2 + y^2 <= 4  (1/2 [x y] [I] [x y]^T - 2 <= 0)

      double[] minimizeF = new double[] {-1.0, -1.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // inequalities
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[1];
      double[][] PMatrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
      double[] qVector = new double[]{0.0, 0.0};
      double r = -2;
      
      inequalities[0] = new PSDQuadraticMultivariateRealFunction(PMatrix, qVector, r);    // x^2+y^2 <= 4

      double[] solution = solveOptimizationProblem(objectiveFunction, inequalities);

      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(Math.sqrt(2.0), solution[0], 1e-5);
      assertEquals(Math.sqrt(2.0), solution[1], 1e-5);
   }
	
	/**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testLinearCostFullyLinearConstrainedEqualityOptimizationProblem() throws Exception
   {
      // Minimize x subject to x+y=4 and x-y=2. Should return (3,1).
      double[] minimizeF = new double[] {1.0, 0.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}, {1.0, -1.0}};
      double[] equalityBVector = new double[]{4.0, 2.0};
      
      double[] solution = solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector);

      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      assertEquals(3.0, solution[0], 1e-5);
      assertEquals(1.0, solution[1], 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 3000)
   public void testZeroCostLinearEqualityOptimizationProblem() throws Exception
   {
      // Minimize 0 subject to x+y=4. Should return any feasible solution.
      double[] minimizeF = new double[] {0.0, 0.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4.0};
      
      double[] solution = solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector);

      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      assertEquals(4.0, solution[0] + solution[1], 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 3000)
   public void testLinearCostLinearEqualityQuadraticInequalityOptimizationProblem() throws Exception
   {
      // Minimize x subject to x+y=4 and y >= x^2. Answer should be ((-1-sqrt(17))/2, (9+sqrt(17))/2))
      double[] minimizeF = new double[] {1.0, 0.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4.0};
      
      // inequalities
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[1];
      double[][] PMatrix = new double[][]{{2.0, 0.0}, {0.0, 0.0}};
      double[] qVector = new double[]{0.0, -1.0};
      double r = 0.0;
      
      inequalities[0] = new PSDQuadraticMultivariateRealFunction(PMatrix, qVector, r);    // x^2 - y <= 0

      double[] solution = solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector, inequalities);

      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(1.0/2.0 * (-1.0 - Math.sqrt(17.0)), solution[0], 1e-5);
      assertEquals(1.0/2.0 * (9.0 + Math.sqrt(17.0)), solution[1], 1e-5);
   }

	/**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testQuadraticCostLinearEqualityQuadraticInequalityOptimizationProblem() throws Exception
   {
      // Minimize -x^2 subject to x+y=4 and y >= x^2. Answer should be ((-1-sqrt(17))/2, (9+sqrt(17))/2))

      double[][] costPMatrix = new double[][]{{-2.0, 0.0},{0.0, 0.0}};
      double[] costQVector = new double[]{0.0, 0.0};
      double costR = 0.0;
      PSDQuadraticMultivariateRealFunction objectiveFunction = new PSDQuadraticMultivariateRealFunction(costPMatrix, costQVector, costR);

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4};
      
      
      // inequalities
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[1];
      double[][] PMatrix = new double[][]{{2.0, 0.0}, {0.0, 0.0}};
      double[] qVector = new double[]{0.0, -1.0};
      double r = 0.0;
      
      inequalities[0] = new PSDQuadraticMultivariateRealFunction(PMatrix, qVector, r);    // x^2 - y <= 0

      double[] solution = solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector, inequalities);

      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(1.0/2.0 * (-1.0 - Math.sqrt(17.0)), solution[0], 1e-5);
      assertEquals(1.0/2.0 * (9.0 + Math.sqrt(17.0)), solution[1], 1e-5);
   }

   private double[] solveOptimizationProblem(LinearMultivariateRealFunction objectiveFunction, double[][] equalityAMatrix, double[] equalityBVector) throws Exception
   {
      return solveOptimizationProblem(objectiveFunction, equalityAMatrix, equalityBVector, null);
   }
   
   private double[] solveOptimizationProblem(ConvexMultivariateRealFunction objectiveFunction, ConvexMultivariateRealFunction[] inequalities) throws Exception
   {
      return solveOptimizationProblem(objectiveFunction, null, null, inequalities);
   }
   
   private double[] solveOptimizationProblem(ConvexMultivariateRealFunction objectiveFunction, double[][] equalityAMatrix, double[] equalityBVector, ConvexMultivariateRealFunction[] inequalities) throws Exception
   {
      // optimization problem
      OptimizationRequest optimizationRequest = new OptimizationRequest();
      optimizationRequest.setF0(objectiveFunction);

      if (equalityAMatrix != null)
      {
         optimizationRequest.setA(equalityAMatrix);
      }
      
      if (equalityBVector != null)
      {
         optimizationRequest.setB(equalityBVector);
      }
      
      if (inequalities != null)
      {
         optimizationRequest.setFi(inequalities);
      }
      
      optimizationRequest.setToleranceFeas(1.E-6);
      optimizationRequest.setTolerance(2.E-6);
      optimizationRequest.setMaxIteration(500);

      // optimization
      JOptimizer optimizer = new JOptimizer();
      optimizer.setOptimizationRequest(optimizationRequest);
      optimizer.optimize();
      OptimizationResponse response = optimizer.getOptimizationResponse();

      double[] solution = response.getSolution();

      return solution;
   }

   /**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testASecondOrderLorenzConeProblemUsingSquaring() throws Exception
   {
      // Minimize -(x + y) subject to z <= sqrt(18) and sqrt(x^2 + y^2) <= z. Answer should be (3, 3, sqrt(18))
      // sqrt(x^2+y^2) <= z (The Lorenz Cone) can be cast as a quadratic constraint x^2 + y^2 <= z^2 and a linear constraint z >= 0
      // Note however that the quadratic constraint is not convex. It has a ice-cream cone shape pointing up and a symmetric one pointing down.
      // The constraint z >= 0 cuts off the bottom pointing down cone.
      // Boyd mentions this approach, but states that it is not valid since each constraint has to be convex on its own accord. 
      // Below we see that the PMatrix is not positive semi-definite, thus not a convex quadratic constraint.
      // Nevertheless, JOptimizer returns the correct solution. Not sure how to do this otherwise using JOptimizer...
      
      double[] minimizeF = new double[] {-1.0, -1.0, 0.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      // inequalities
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[3];
      double[][] PMatrix = new double[][]{{2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, -2.0}};
      double[] qVector = new double[]{0.0, 0.0, 0.0};
      double r = 0.0;
      
      inequalities[0] = new PSDQuadraticMultivariateRealFunction(PMatrix, qVector, r);    

      qVector = new double[]{0.0, 0.0, 1.0};
      r = -Math.sqrt(18.0);
      inequalities[1] = new LinearMultivariateRealFunction(qVector, r);    
      
      qVector = new double[]{0.0, 0.0, -1.0};
      r = 0.0;
      inequalities[2] = new LinearMultivariateRealFunction(qVector, r);    
      
      OptimizationRequest optimizationRequest = new OptimizationRequest();
      optimizationRequest.setF0(objectiveFunction);

      optimizationRequest.setFi(inequalities);
      optimizationRequest.setToleranceFeas(1.E-6);
      optimizationRequest.setTolerance(2.E-6);
      optimizationRequest.setMaxIteration(500);

      // optimization
//      JOptimizer optimizer = new JOptimizer();
      
      int numberOfTests = 2000;
      
      double[] solution = null;
      
      long startTime = System.currentTimeMillis();
      for (int i=0; i<numberOfTests; i++)
      {
         JOptimizer optimizer = new JOptimizer();

         optimizer.setOptimizationRequest(optimizationRequest);
         optimizer.optimize();
         OptimizationResponse response = optimizer.getOptimizationResponse();

         solution = response.getSolution();
      }
      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;
      double timePerSolve = totalTime / numberOfTests;
      
      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ", " + solution[2] + ")");
      if (VERBOSE) System.out.println("timePerSolve = " + (timePerSolve * 1000.0) + " milliseconds");
      
      assertEquals(3.0, solution[0], 1e-5);
      assertEquals(3.0, solution[1], 1e-5);
      assertEquals(Math.sqrt(18.0), solution[2], 1e-5);
   }
   
   
   /**
    * JOptimizer has not been properly implemented and very simple tests fail
    * 
    * @throws Exception
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testASecondOrderLorenzConeProblemUsingSOCP() throws Exception
   {
      // Minimize -(x + y) subject to z <= sqrt(18) and sqrt(x^2 + y^2) <= z. Answer should be (3, 3, sqrt(18))
      // Use SOCP constraints directly.
      
      double[] minimizeF = new double[] {-1.0, -1.0, 0.0};
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(minimizeF, 0.0);

      double[][] socpAMatrix = new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};
      double[] socpBVector = new double[]{0.0, 0.0, 0.0};
      double[] socpCVector = new double[]{0.0, 0.0, 1.0};
      double socpDScalar = 0.0;
      
      int socpConstraintDimension = 1;
      ArrayList<SOCPConstraintParameters> socpConstraintParametersList = new ArrayList<SOCPConstraintParameters>(socpConstraintDimension);
      SOCPLogarithmicBarrier barrierFunction = new SOCPLogarithmicBarrier(socpConstraintParametersList, socpConstraintDimension);
      SOCPConstraintParameters socpConstraintParameters = barrierFunction.new SOCPConstraintParameters(socpAMatrix, socpBVector, socpCVector, socpDScalar);
      socpConstraintParametersList.add(socpConstraintParameters);      
      
      // inequalities
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[1];
      inequalities[0] = new LinearMultivariateRealFunction(new double[] {0.0, 0.0, 1.0}, -Math.sqrt(18.0));    // z <= sqrt(18.0)

      OptimizationRequest optimizationRequest = new OptimizationRequest();
      optimizationRequest.setF0(objectiveFunction);

      optimizationRequest.setFi(inequalities);
      optimizationRequest.setToleranceFeas(1.E-6);
      optimizationRequest.setTolerance(2.E-6);
      optimizationRequest.setMaxIteration(500);

      // optimization
      BarrierMethod optimizer = new BarrierMethod(barrierFunction);

      optimizer.setOptimizationRequest(optimizationRequest);
      optimizer.optimize();
      OptimizationResponse response = optimizer.getOptimizationResponse();

      double[] solution = response.getSolution();

      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ", " + solution[2] + ")");

      assertEquals(3.0, solution[0], 1e-5);
      assertEquals(3.0, solution[1], 1e-5);
      assertEquals(Math.sqrt(18.0), solution[2], 1e-5);
   }
}
