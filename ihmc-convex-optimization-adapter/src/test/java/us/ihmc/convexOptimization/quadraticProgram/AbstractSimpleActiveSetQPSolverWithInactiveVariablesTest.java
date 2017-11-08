package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import static org.junit.Assert.*;

public abstract class AbstractSimpleActiveSetQPSolverWithInactiveVariablesTest extends AbstractSimpleActiveSetQPSolverTest
{
   private static final boolean VERBOSE = false;

   public abstract ActiveSetQPSolverWithInactiveVariablesInterface createSolverToTest();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithInequalityConstraintsAndInactiveVariables()
   {
      testSimpleCasesWithInequalityConstraintsAndInactiveVariables(1);
   }

   public void testSimpleCasesWithInequalityConstraintsAndInactiveVariables(int expectedNumberOfIterations)
   {
      ActiveSetQPSolverWithInactiveVariablesInterface solver = createSolverToTest();

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
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(expectedNumberOfIterations, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34 subject to x <= 7 y <= 1, with y inactive
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] activeVariables = new double[] {1.0, 0.0};
      solver.setActiveVariables(activeVariables);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, { 0.0, 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 7.0, 1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[2];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(expectedNumberOfIterations, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(5.0, solution[0], 1e-7);
      assertEquals(0.0, solution[1], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[1], 1e-7);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(9.0, objectiveCost, 1e-7);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34 subject to x <= 7 y <= 1, with x inactive
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      activeVariables = new double[] {0.0, 1.0};
      solver.setActiveVariables(activeVariables);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, { 0.0, 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 7.0, 1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[0];
      lagrangeInequalityMultipliers = new double[2];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(expectedNumberOfIterations + 1, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(4.0, lagrangeInequalityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[1], 1e-7);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(29.0, objectiveCost, 1e-7);


      // Minimize x^2 + y^2 subject to x + y = 1.0, x <= y - 1 (x - y <= -1.0), x inactive
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 1.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      activeVariables = new double[] { 1.0, 0.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);

      assertEquals(2, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));

      // Minimize x^2 + y^2 subject to x + y = 1.0, x <= y - 1 (x - y <= -1.0), y inactive
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      linearEqualityConstraintsBVector = new double[] { 1.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      activeVariables = new double[] { 0.0, 1.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertEquals(expectedNumberOfIterations, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(0.0 , solution[0], 1e-7);
      assertEquals(1.0 , solution[1], 1e-7);
      assertEquals(-2.0, lagrangeEqualityMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 2.0, 3x - 3y = 0.0, x <= 2, x <= 10, y <= 3, x active
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 }, { 3.0, -3.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0, 0.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 2.0, 10.0, 3.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      activeVariables = new double[] { 1.0, 0.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[2];
      lagrangeInequalityMultipliers = new double[3];
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      assertTrue(numberOfIterations <= 1);

      assertEquals(2, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000000)
   public void testSimpleCasesWithBoundsConstraintsAndInactiveVariables()
   {
      testSimpleCasesWithBoundsConstraintsAndInactiveVariables(1, 2, 3, 2, false);
   }

   public void testSimpleCasesWithBoundsConstraintsAndInactiveVariables(int expectedNumberOfIterations, int expectedNumberOfIterations2, int expectedNubmerOfIterations3,
                                                    int expectedNumberOfIterations4, boolean ignoreLagrangeMultipliers)
   {
      ActiveSetQPSolverWithInactiveVariablesInterface solver = createSolverToTest();

      // Minimize x^T * x
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] variableLowerBounds = new double[] { Double.NEGATIVE_INFINITY };
      double[] variableUpperBounds = new double[] { Double.POSITIVE_INFINITY };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      double[] solution = new double[1];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[0];
      double[] lagrangeLowerBoundMultipliers = new double[1];
      double[] lagrangeUpperBoundMultipliers = new double[1];

      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
      assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);

      // minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 1 <= y <= 10, -2 <= z, y and z active
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 1.0, -2.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      double[] activeVariables = new double[] { 0.0, 1.0, 1.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations2, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(2.0, solution[1], 1e-7);
      assertEquals(10.0, solution[2], 1e-7);
      if (!ignoreLagrangeMultipliers)
      {
         assertEquals(-24.0, lagrangeEqualityMultipliers[0], 1e-7);
         assertEquals(20.0, lagrangeInequalityMultipliers[0], 1e-7);

         assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[2], 1e-7);

         assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[2], 1e-7);
      }

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(104.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, -2 <= z, x and z active
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, -2.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      activeVariables = new double[] { 1.0, 0.0, 1.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations2, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(2.0, solution[0], 1e-7);
      assertEquals(0.0, solution[1], 1e-7);
      assertEquals(8.0, solution[2], 1e-7);
      if (!ignoreLagrangeMultipliers)
      {
         assertEquals(-4.0, lagrangeEqualityMultipliers[0], 1e-7);
         assertEquals(16.0, lagrangeInequalityMultipliers[0], 1e-7);

         assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[2], 1e-7);

         assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[2], 1e-7);
      }

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(68.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, -2 <= z, x and y active
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, -2.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      activeVariables = new double[] { 1.0, 1.0, 0.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations2, numberOfIterations);

      assertEquals(3, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isNaN(solution[2]));

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, -2 <= z, x active
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, -2.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      activeVariables = new double[] { 1.0, 0.0, 0.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(2.0, solution[0], 1e-7);
      assertEquals(0.0, solution[1], 1e-7);
      assertEquals(0.0, solution[2], 1e-7);
      if (!ignoreLagrangeMultipliers)
      {
         assertEquals(-4.0, lagrangeEqualityMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeInequalityMultipliers[0], 1e-7);

         assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[2], 1e-7);

         assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[2], 1e-7);
      }

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(4.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, -2 <= z, y active
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, -2.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      activeVariables = new double[] { 0.0, 1.0, 0.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations4, numberOfIterations);

      assertEquals(3, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isNaN(solution[2]));

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, -2 <= z, z active
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, -2.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      activeVariables = new double[] { 0.0, 0.0, 1.0 };
      solver.setActiveVariables(activeVariables);

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNumberOfIterations2, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(0.0, solution[0], 1e-7);
      assertEquals(0.0, solution[1], 1e-7);
      assertEquals(8.0, solution[2], 1e-7);
      if (!ignoreLagrangeMultipliers)
      {
         assertEquals(0.0, lagrangeEqualityMultipliers[0], 1e-7);
         assertEquals(16.0, lagrangeInequalityMultipliers[0], 1e-7);

         assertEquals(0.0, lagrangeLowerBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeLowerBoundMultipliers[2], 1e-7);

         assertEquals(0.0, lagrangeUpperBoundMultipliers[0], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[1], 1e-7);
         assertEquals(0.0, lagrangeUpperBoundMultipliers[2], 1e-7);
      }

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(64.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, 3 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { 3.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      solution = new double[3];
      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[1];
      lagrangeLowerBoundMultipliers = new double[3];
      lagrangeUpperBoundMultipliers = new double[3];

      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lagrangeLowerBoundMultipliers, lagrangeUpperBoundMultipliers);
      assertEquals(expectedNubmerOfIterations3, numberOfIterations);

      assertEquals(3, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isNaN(solution[2]));

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertTrue(Double.isNaN(objectiveCost));
   }



   private void verifyEqualityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertEquals(0.0, maxAbsoluteError, 1e-5);
   }

   private void verifyInequalityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxSignedError = getMaxInequalityConstraintError(numberOfEqualityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxSignedError < 1e-10);
   }

   private void verifyEqualityConstraintsDoNotHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertTrue(maxAbsoluteError > 1e-5);
   }

   private void verifyInequalityConstraintsDoNotHold(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxError = getMaxInequalityConstraintError(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxError > 1e-5);
   }

   private void verifyVariableBoundsHold(int testNumber, DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds, DenseMatrix64F solution)
   {
      for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
      {
         assertTrue("In test number " + testNumber + " the solution " + solution.get(i, 0) + " is less than the lower bound " + variableLowerBounds.get(i, 0), solution.get(i, 0) >= variableLowerBounds.get(i, 0) - 1e-7);
      }

      for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
      {
         assertTrue(solution.get(i, 0) <= variableUpperBounds.get(i, 0) + 1e-7);
      }
   }

   private double getMaxEqualityConstraintError(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearEqualityConstraintsBVector);

      return getMaxAbsoluteDataEntry(checkMatrix);
   }

   private double getMaxInequalityConstraintError(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearInequalityConstraintsDVector);

      return getMaxSignedDataEntry(checkMatrix);
   }

   private DenseMatrix64F projectOntoEqualityConstraints(DenseMatrix64F solutionMatrix, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
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
