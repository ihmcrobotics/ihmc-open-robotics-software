package us.ihmc.convexOptimization;

import static org.junit.Assert.assertEquals;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import org.yaml.snakeyaml.Yaml;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public abstract class ConvexOptimizationAdapterTest
{
   public abstract ConvexOptimizationAdapter createConvexOptimizationAdapter();
   public abstract double getTestErrorEpsilon();
   
   @SuppressWarnings("unchecked")
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void qpsFileTest() throws IOException
   {

      DenseMatrix64F beq = new DenseMatrix64F(0,0);
      DenseMatrix64F Aeq = new DenseMatrix64F(0,0);
      DenseMatrix64F b = new DenseMatrix64F(0,0);
      DenseMatrix64F A = new DenseMatrix64F(0,0);
      DenseMatrix64F H = new DenseMatrix64F(0,0);
      DenseMatrix64F f = new DenseMatrix64F(0,0);
      
      File projectDirectory = new File(new File("").getAbsolutePath());
      File yamlQpProblemDirectory = new File(projectDirectory, "/Matlab/YamlQpProblems");
      File[] yamlQpProblemFileList = yamlQpProblemDirectory.listFiles();
      
      Yaml yaml = new Yaml();
      
      for(int i = 0; i < yamlQpProblemFileList.length; i++)
      {
         InputStream input = new FileInputStream(yamlQpProblemFileList[i]);
      
         Map<String, Object> object = (Map<String, Object>) yaml.load(input);
         System.out.print(object + "\n");
         
         MatrixTools.yamlFieldToMatrix(beq,"beq",object);
         MatrixTools.yamlFieldToMatrix(Aeq,"Aeq",object);
         MatrixTools.yamlFieldToMatrix(A,"A",object);
         MatrixTools.yamlFieldToMatrix(b,"b",object);
         MatrixTools.yamlFieldToMatrix(H,"H",object);
         MatrixTools.yamlFieldToMatrix(f,"f",object);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testASimpleRedundantEqualityCase()
   {
      // Minimize x subject to x = 2 and x = 2;
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0});
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(new double[][]{{1.0},{1.0}});
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(new double[]{2.0,2.0});
      
      double[] solution = convexOptimizationAdapter.solve();
      
      assertEquals(1, solution.length);
      assertEquals(2.0, solution[0], getTestErrorEpsilon());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testASimpleRedundantEqualityCase2d()
   {
	   // Minimize x + y subject to x + 2y = 2, 2x + 4y = 4, 3x + 7y = 7; Solution is (0, 1)
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0,1.0});
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(new double[][]{{1,2},{2,4},{3,7}});
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(new double[]{2,4,7});
      
      double[] solution = convexOptimizationAdapter.solve();
      
      assertEquals(2, solution.length);
      assertEquals(0.0, solution[0], getTestErrorEpsilon());
      assertEquals(1.0, solution[1], getTestErrorEpsilon());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
  public void JOptimizerWebpageLPExample() throws Exception
  {
      //from http://www.joptimizer.com/linearProgramming.html
      
      // Objective function (plane)
      double[] qVector = new double[] { -1., -1. };
      double r= 4;
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(qVector, r);

      //inequalities (polyhedral feasible set G.X<H )
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[4];
      double[][] G = new double[][] {{4./3., -1}, {-1./2., 1.}, {-2., -1.}, {1./3., 1.}};
      double[] h = new double[] {2., 1./2., 2., 1./2.};
      inequalities[0] = new LinearMultivariateRealFunction(G[0], -h[0]);
      inequalities[1] = new LinearMultivariateRealFunction(G[1], -h[1]);
      inequalities[2] = new LinearMultivariateRealFunction(G[2], -h[2]);
      inequalities[3] = new LinearMultivariateRealFunction(G[3], -h[3]);
      
      //optimization problem
      OptimizationRequest or = new OptimizationRequest();
      or.setF0(objectiveFunction);
      or.setFi(inequalities);
      or.setToleranceFeas(1.E-9);
      or.setTolerance(1.E-9);
      
      //optimization
      JOptimizer opt = new JOptimizer();
      opt.setOptimizationRequest(or);

      int returnCode = opt.optimize();
      double[] sol=opt.getOptimizationResponse().getSolution();
      assertEquals(1.5, sol[0], 1e-7);
      assertEquals(0, sol[1], 1e-7);
      
      
      //repeat with out adapter
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(qVector);
      convexOptimizationAdapter.setLinearInequalityConstraints(G, h); //-x <= -2.0
      
      double[] sol2 = convexOptimizationAdapter.solve();
      
      assertEquals(2, sol2.length);
      assertEquals(1.5, sol2[0], 1e-5);
      assertEquals(0, sol2[1], 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testASimpleInequalityCase() throws Exception
   {
      // Minimize x subject to -x <= -2; (x >= 2)
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0});
      convexOptimizationAdapter.setLinearInequalityConstraints(new double[][]{{-1.0}}, new double[]{-2.0}); //-x <= -2.0
      
      double[] solution = convexOptimizationAdapter.solve();
      
      assertEquals(1, solution.length);
      assertEquals(2.0, solution[0], 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testASimpleMaximizationInequalityCase()
   {
      // Minimize -x subject to x <= 5
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();

      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{-1.0});
      convexOptimizationAdapter.setLinearInequalityConstraints(new double[][]{{1.0}}, new double[]{5.0}); // 1.0 x <= 5.0

      double[] solution = convexOptimizationAdapter.solve();

      assertEquals(5.0, solution[0], 1e-5);
   }
   
   // Need to implement addQuadraticInequalities
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testLinearCostQuadraticInequalityOptimizationProblem()
   {
      // Minimize -x-y subject to x^2 + y^2 <= 4  (1/2 [x y] [I] [x y]^T - 2 <= 0)
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{-1.0, -1.0});

      // Quadratic Inequalities
      double[][] PMatrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
      double[] qVector = new double[]{0.0, 0.0};
      double r = -2;
      
      convexOptimizationAdapter.addQuadraticInequalities(PMatrix, qVector, r);

      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(Math.sqrt(2.0), solution[0], 1e-5);
      assertEquals(Math.sqrt(2.0), solution[1], 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearCostFullyLinearConstrainedEqualityOptimizationProblem()
   {
      // Minimize x subject to x+y=4 and x-y=2. Should return (3,1).
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0, 0.0});

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}, {1.0, -1.0}};
      double[] equalityBVector = new double[]{4.0, 2.0};
      
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(equalityAMatrix);
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(equalityBVector);
      
      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      assertEquals(3.0, solution[0], getTestErrorEpsilon());
      assertEquals(1.0, solution[1], getTestErrorEpsilon());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testZeroCostLinearEqualityOptimizationProblem() throws Exception
   {
      // Minimize 0 subject to x+y=4. Should return any feasible solution.
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{0.0, 0.0});

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4.0};
      
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(equalityAMatrix);
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(equalityBVector);
      
      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      assertEquals(4.0, solution[0] + solution[1], getTestErrorEpsilon());
   }
   
   //Not implemented yet!
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testLinearCostLinearEqualityQuadraticInequalityOptimizationProblem() throws Exception
   {
      // Minimize x subject to x+y=4 and y >= x^2. Answer should be ((-1-sqrt(17))/2, (9+sqrt(17))/2))
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{1.0, 0.0});

      // Equalities:
      double[][] equalityAMatrix = new double[][]{{1.0, 1.0}};
      double[] equalityBVector = new double[]{4.0};
      
      convexOptimizationAdapter.setLinearEqualityConstraintsAMatrix(equalityAMatrix);
      convexOptimizationAdapter.setLinearEqualityConstraintsBVector(equalityBVector);
      
      // inequalities
      double[][] pMatrix = new double[][]{{2.0, 0.0}, {0.0, 0.0}};
      double[] qVector = new double[]{0.0, -1.0};
      double r = 0.0;
      convexOptimizationAdapter.addQuadraticInequalities(pMatrix, qVector, r);
      
      double[] solution = convexOptimizationAdapter.solve();

//      if (VERBOSE) System.out.println("solution = (" + solution[0] + ", " + solution[1] + ")");
      
      assertEquals(1.0/2.0 * (-1.0 - Math.sqrt(17.0)), solution[0], 1e-5);
      assertEquals(1.0/2.0 * (9.0 + Math.sqrt(17.0)), solution[1], 1e-5);
   }

   //Not implemented yet!
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testASecondOrderLorenzConeProblemUsingSOCP() throws Exception
   {
      // Minimize -(x + y) subject to z <= sqrt(18) and sqrt(x^2 + y^2) <= z. Answer should be (3, 3, sqrt(18))
      ConvexOptimizationAdapter convexOptimizationAdapter = createConvexOptimizationAdapter();
      convexOptimizationAdapter.setLinearCostFunctionVector(new double[]{-1.0, -1.0, 0.0});

      double[][] secondOrderConeAMatrix = new double[][]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};
      double[] secondOrderConeBVector = new double[]{0.0,0.0,0.0};
      double[] secondOrderConeCVector = new double[]{0.0, 0.0, 1.0};
      double secondOrderConeDScalar = 0.0;
      
      convexOptimizationAdapter.addSecondOrderConeConstraints(secondOrderConeAMatrix, secondOrderConeBVector, secondOrderConeCVector, secondOrderConeDScalar);

      // inequalities
      convexOptimizationAdapter.setLinearInequalityConstraints(new double[][]{{0.0, 0.0, 1.0}}, new double[]{Math.sqrt(18.0)}); // z <= sqrt(18.0)

      double[] solution = convexOptimizationAdapter.solve();

      System.out.println("solution = (" + solution[0] + ", " + solution[1] + ", " + solution[2] + ")");

      assertEquals(3.0, solution[0], 1e-5);
      assertEquals(3.0, solution[1], 1e-5);
      assertEquals(Math.sqrt(18.0), solution[2], 1e-5);
   }

}
