package us.ihmc.convexOptimization.quadraticProgram;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class SimpleInefficientActiveSetQPSolverTest
{
   private static final boolean VERBOSE = false;

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithNoInequalityConstraints()
   {
      SimpleInefficientActiveSetQPSolver solver = new SimpleInefficientActiveSetQPSolver();

      // Minimize x^T * x
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] solution = new double[1];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[0];
      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);
      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);

      // Minimize (x-5) * (x-5) = x^2 - 10x + 25
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { -10.0 };
      quadraticCostScalar = 25.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(5.0, solution[0], 1e-7);
      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, 1e-7);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[0];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(5.0, solution[0], 1e-7);
      assertEquals(3.0, solution[1], 1e-7);
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 1.0
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 1.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);
      
      assertEquals(0.5, solution[0], 1e-7);
      assertEquals(0.5, solution[1], 1e-7);
      assertEquals(-1.0, lagrangeEqualityMultipliers[0], 1e-7); // Lagrange multiplier is -1.0;
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.5, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 2.0, 3x - 3y = 0.0
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 }, { 3.0, -3.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0, 0.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[2];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(-2.0, lagrangeEqualityMultipliers[0], 1e-7); // Lagrange multiplier
      assertEquals(0.0, lagrangeEqualityMultipliers[1], 1e-7); // Lagrange multiplier
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, 1e-7);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithInequalityConstraints()
   {
      SimpleInefficientActiveSetQPSolver solver = new SimpleInefficientActiveSetQPSolver();

      // Minimize x^T * x subject to x <= 1
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { 1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      double[] solution = new double[1];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[1];
      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);
      
      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);
      
      
      // Minimize x^T * x subject to x >= 1 (-x <= -1)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(2, numberOfIterations);
      
      assertEquals(1, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(2.0, lagrangeInequalityMultipliers[0], 1e-7);

      // Minimize (x-5) * (x-5) = x^2 - 10x + 25 subject to x <= 3.0
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { -10.0 };
      quadraticCostScalar = 25.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 3.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[1];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(2, numberOfIterations);
      
      assertEquals(1, solution.length);
      assertEquals(3.0, solution[0], 1e-7);
      assertEquals(4.0, lagrangeInequalityMultipliers[0], 1e-7);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(4.0, objectiveCost, 1e-7);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34 subject to x <= 7 y <= 1
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, {0.0, 1.0} };
      linearInqualityConstraintsDVector = new double[] { 7.0, 1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[2];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[2];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(2, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(5.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(4.0, lagrangeInequalityMultipliers[1], 1e-7);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(4.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 1.0, x <= y - 1 (x - y <= -1.0)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 1.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, -1.0 }};
      linearInqualityConstraintsDVector = new double[] { -1.0};
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[2];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(2, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(-1.0, lagrangeEqualityMultipliers[0], 1e-7);
      assertEquals(1.0, lagrangeInequalityMultipliers[0], 1e-7);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(1.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 2.0, 3x - 3y = 0.0, x <= 2, x <= 10, y <= 3
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 }, { 3.0, -3.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0, 0.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, {1.0, 0.0}, {0.0, 1.0}};
      linearInqualityConstraintsDVector = new double[] { 2.0, 10.0, 3.0};
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[2];
      lagrangeEqualityMultipliers = new double[2];
      lagrangeInequalityMultipliers = new double[3];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(1, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(-2.0, lagrangeEqualityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeEqualityMultipliers[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[2], 1e-7);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, 1e-7);
   }

   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test2DCasesWithPolygonConstraints()
   {
      SimpleInefficientActiveSetQPSolver solver = new SimpleInefficientActiveSetQPSolver();

      // Minimize x^2 + y^2 subject to 3 <= x <= 5, 2 <= y <= 4
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, {0.0, 2.0} };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 } , { -1.0, 0.0 } , { 0.0, 1.0 } , { 0.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { 5.0, -3.0, 4.0, -2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      double[] solution = new double[2];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[4];
      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(2, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(3.0, solution[0], 1e-7);
      assertEquals(2.0, solution[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(6.0, lagrangeInequalityMultipliers[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[2], 1e-7);
      assertEquals(4.0, lagrangeInequalityMultipliers[3], 1e-7);

      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2)
      // Equality solution will violate both constraints, but optimal only has the first constraint active.
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, {0.0, 2.0} };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { -1.0, -1.0 } , { -10.0, 1.0 }};
      linearInqualityConstraintsDVector = new double[] { -2.0, -2.0};
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[2];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[2];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(3, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(2.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[1], 1e-7);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChallengingCasesWithPolygonConstraints()
   {
      SimpleInefficientActiveSetQPSolver solver = new SimpleInefficientActiveSetQPSolver();
      solver.setMaxNumberOfIterations(10);
      
      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2), 
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, {0.0, 2.0} };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { -1.0, -1.0 } , { -10.0, 1.0 }, { 1.0, -10.0 }};
      double[] linearInqualityConstraintsDVector = new double[] { -2.0, -2.0, -2.0};
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      double[] solution = new double[2];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[3];
      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(3, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(2.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[2], 1e-7);
      
      // Reorder and make sure it works:
      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2), 
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, {0.0, 2.0} };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { -10.0, 1.0 }, { -1.0, -1.0 } , { 1.0, -10.0 }};
      linearInqualityConstraintsDVector = new double[] { -2.0, -2.0, -2.0};
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      solution = new double[2];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[3];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(3, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(2.0, lagrangeInequalityMultipliers[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[2], 1e-7);
   }
   
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCaseWithNoSolution()
   {
      SimpleInefficientActiveSetQPSolver solver = new SimpleInefficientActiveSetQPSolver();
      int maxNumberOfIterations = 10;
      solver.setMaxNumberOfIterations(maxNumberOfIterations);
      
      // Minimize x^2 + y^2 subject to x + y = 5, x + y <= 2 
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, {0.0, 2.0} };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 }};
      double[] linearEqualityConstraintsBVector = new double[] { 5.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
      
      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { 2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      
      double[] solution = new double[2];
      double[] lagrangeEqualityMultipliers = new double[1];
      double[] lagrangeInequalityMultipliers = new double[1];
      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(maxNumberOfIterations, numberOfIterations);
      
      assertEquals(2, solution.length);
      assertEquals(Double.NaN, solution[0], 1e-7);
      assertEquals(Double.NaN, solution[1], 1e-7);
      assertTrue(Double.isInfinite(lagrangeEqualityMultipliers[0]));
      assertTrue(Double.isInfinite(lagrangeInequalityMultipliers[0]));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLargeRandomProblemWithInequalityConstraints()
   {
      Random random = new Random(1776L);

      SimpleInefficientActiveSetQPSolver solver = new SimpleInefficientActiveSetQPSolver();

      int numberOfTests = 100;

      long startTimeMillis = System.currentTimeMillis();
      int maxNumberOfIterations = 0;
      
      for (int testNumber = 0; testNumber < numberOfTests; testNumber++)
      {
         solver.clear();
         int numberOfVariables = 80;
         int numberOfEqualityConstraints = 10;
         int numberOfInequalityConstraints = 36;

         DenseMatrix64F costQuadraticMatrix = RandomTools.generateRandomMatrix(random, numberOfVariables, numberOfVariables);
         DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
         CommonOps.scale(numberOfVariables, identity);
         CommonOps.addEquals(costQuadraticMatrix, identity);

         DenseMatrix64F costLinearVector = RandomTools.generateRandomMatrix(random, numberOfVariables, 1);
         double quadraticCostScalar = RandomTools.generateRandomDouble(random, 30.0);

         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

         DenseMatrix64F linearEqualityConstraintsAMatrix = RandomTools.generateRandomMatrix(random, numberOfEqualityConstraints, numberOfVariables);
         DenseMatrix64F linearEqualityConstraintsBVector = RandomTools.generateRandomMatrix(random, numberOfEqualityConstraints, 1);
         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
         
         DenseMatrix64F linearInequalityConstraintsCMatrix = RandomTools.generateRandomMatrix(random, numberOfInequalityConstraints, numberOfVariables);
         DenseMatrix64F linearInequalityConstraintsDVector = RandomTools.generateRandomMatrix(random, numberOfInequalityConstraints, 1);
         solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);

         double[] solution = new double[numberOfVariables];
         double[] lagrangeEqualityMultipliers = new double[numberOfEqualityConstraints];
         double[] lagrangeInequalityMultipliers = new double[numberOfInequalityConstraints];
         int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
         if (numberOfIterations > maxNumberOfIterations) maxNumberOfIterations = numberOfIterations;
//         System.out.println("numberOfIterations = " + numberOfIterations);
         
         assertEquals(numberOfVariables, solution.length);
         assertEquals(numberOfEqualityConstraints, lagrangeEqualityMultipliers.length);

         DenseMatrix64F solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);
         solutionMatrix.setData(solution);
         double objectiveCost = solver.getObjectiveCost(solutionMatrix);

         // Verify constraints hold:
         verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
         verifyInequalityConstraintsHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);

         // Verify objective is minimized by comparing to small perturbation:
         double[] solutionWithSmallPerturbation = new double[numberOfVariables];
         for (int i = 0; i < numberOfVariables; i++)
         {
            solutionWithSmallPerturbation[i] = solution[i] + RandomTools.generateRandomDouble(random, 1e-4);
         }

         solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);
         solutionMatrix.setData(solutionWithSmallPerturbation);

         verifyEqualityConstraintsDoNotHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
         verifyInequalityConstraintsDoNotHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
         
         
         int activeSetSize = 0;
         for (int i=0; i<numberOfInequalityConstraints; i++)
         {
            double lagrangeMultiplier = lagrangeInequalityMultipliers[i];
            
            if (lagrangeMultiplier < 0.0)
            {
               throw new RuntimeException();
            }
            if (lagrangeMultiplier > 0.0)
            {
               activeSetSize++;
            }
         }
         
         DenseMatrix64F augmentedLinearEqualityConstraintsAMatrix = new DenseMatrix64F(numberOfEqualityConstraints + activeSetSize, numberOfVariables);
         DenseMatrix64F augmentedLinearEqualityConstraintsBVector = new DenseMatrix64F(numberOfEqualityConstraints + activeSetSize, 1);
         
         CommonOps.extract(linearEqualityConstraintsAMatrix, 0, numberOfEqualityConstraints, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, 0, 0);
         CommonOps.extract(linearEqualityConstraintsBVector, 0, numberOfEqualityConstraints, 0, 1, augmentedLinearEqualityConstraintsBVector, 0, 0);
         
         int index = 0;
         for (int i=0; i<numberOfInequalityConstraints; i++)
         {
            double lagrangeMultiplier = lagrangeInequalityMultipliers[i];
            
            if (lagrangeMultiplier < 0.0)
            {
               throw new RuntimeException();
            }
            if (lagrangeMultiplier > 0.0)
            {
               CommonOps.extract(linearInequalityConstraintsCMatrix, i, i + 1, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, numberOfEqualityConstraints + index, 0);
               CommonOps.extract(linearInequalityConstraintsDVector, i, i + 1, 0, 1, augmentedLinearEqualityConstraintsBVector, numberOfEqualityConstraints + index, 0);
               index++;
            }
         }
         
         DenseMatrix64F solutionMatrixProjectedOntoEqualityConstraints = projectOntoEqualityConstraints(solutionMatrix, augmentedLinearEqualityConstraintsAMatrix,
               augmentedLinearEqualityConstraintsBVector);
         verifyEqualityConstraintsHold(numberOfEqualityConstraints + activeSetSize, augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector,
               solutionMatrixProjectedOntoEqualityConstraints);

         double objectiveCostWithSmallPerturbation = solver.getObjectiveCost(solutionMatrixProjectedOntoEqualityConstraints);

         assertTrue("objectiveCostWithSmallPerturbation = " + objectiveCostWithSmallPerturbation + ", objectiveCost = " + objectiveCost,
               objectiveCostWithSmallPerturbation > objectiveCost);
      }

      long endTimeMillis = System.currentTimeMillis();

      double timePerTest = ((double) (endTimeMillis - startTimeMillis)) * 0.001 / ((double) numberOfTests);
      if (VERBOSE) 
      {
         System.out.println("Time per test is " + timePerTest);
         System.out.println("maxNumberOfIterations is " + maxNumberOfIterations);
      }
   }

   private void verifyEqualityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix,
         DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector,
            solutionMatrix);
      assertEquals(0.0, maxAbsoluteError, 1e-5);
   }
   
   private void verifyInequalityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix,
         DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxSignedError = getMaxInequalityConstraintError(numberOfEqualityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector,
            solutionMatrix);
      assertTrue(maxSignedError < 1e-10);
   }

   private void verifyEqualityConstraintsDoNotHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix,
         DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector,
            solutionMatrix);
      assertTrue(maxAbsoluteError > 1e-5);
   }
   
   private void verifyInequalityConstraintsDoNotHold(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix,
         DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxError = getMaxInequalityConstraintError(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxError > 1e-5);
   }

   private double getMaxEqualityConstraintError(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix,
         DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearEqualityConstraintsBVector);

      return getMaxAbsoluteDataEntry(checkMatrix);
   }
   
   private double getMaxInequalityConstraintError(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix,
         DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearInequalityConstraintsDVector);

      return getMaxSignedDataEntry(checkMatrix);
   }

   private DenseMatrix64F projectOntoEqualityConstraints(DenseMatrix64F solutionMatrix, DenseMatrix64F linearEqualityConstraintsAMatrix,
         DenseMatrix64F linearEqualityConstraintsBVector)
   {
      int numberOfVariables = solutionMatrix.getNumRows();
      if (linearEqualityConstraintsAMatrix.getNumCols() != numberOfVariables)
         throw new RuntimeException();

      int numberOfConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      if (linearEqualityConstraintsBVector.getNumRows() != numberOfConstraints)
         throw new RuntimeException();

      DenseMatrix64F AZMinusB = new DenseMatrix64F(numberOfConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, AZMinusB);
      CommonOps.subtractEquals(AZMinusB, linearEqualityConstraintsBVector);

      DenseMatrix64F AATransposeInverse = new DenseMatrix64F(numberOfConstraints, numberOfConstraints);
      DenseMatrix64F linearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(linearEqualityConstraintsAMatrix);
      CommonOps.transpose(linearEqualityConstraintsAMatrixTranspose);

      CommonOps.mult(linearEqualityConstraintsAMatrix, linearEqualityConstraintsAMatrixTranspose, AATransposeInverse);
      CommonOps.invert(AATransposeInverse);

      DenseMatrix64F ATransposeAATransposeInverse = new DenseMatrix64F(numberOfVariables, numberOfConstraints);
      CommonOps.mult(linearEqualityConstraintsAMatrixTranspose, AATransposeInverse, ATransposeAATransposeInverse);

      DenseMatrix64F vectorToSubtract = new DenseMatrix64F(numberOfVariables, 1);
      CommonOps.mult(ATransposeAATransposeInverse, AZMinusB, vectorToSubtract);

      DenseMatrix64F projectedSolutionMatrix = new DenseMatrix64F(solutionMatrix);
      CommonOps.subtractEquals(projectedSolutionMatrix, vectorToSubtract);

      return projectedSolutionMatrix;
   }

   private double getMaxAbsoluteDataEntry(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            double absoluteValue = Math.abs(matrix.get(i, j));
            if (absoluteValue > max)
            {
               max = absoluteValue;
            }
         }
      }

      return max;
   }
   

   private double getMaxSignedDataEntry(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            double value = matrix.get(i, j);
            if (value > max)
            {
               max = value;
            }
         }
      }

      return max;
   }

}
