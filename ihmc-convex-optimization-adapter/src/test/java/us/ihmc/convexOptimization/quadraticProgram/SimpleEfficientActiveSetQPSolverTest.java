package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SimpleEfficientActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      SimpleEfficientActiveSetQPSolver simpleEfficientActiveSetQPSolver = new SimpleEfficientActiveSetQPSolver();
      simpleEfficientActiveSetQPSolver.setUseWarmStart(false);
      return simpleEfficientActiveSetQPSolver;

   }
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChallengingCasesWithPolygonConstraintsCheckFailsWithSimpleSolverWithWarmStart()
   {
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();
      solver.setMaxNumberOfIterations(10);
      solver.setUseWarmStart(true);

      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2),
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { -1.0, -1.0 }, { -10.0, 1.0 }, { 1.0, -10.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -2.0, -2.0, -2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      double[] solution = new double[2];
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[3];
      solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
      int numberOfIterations = solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);

      assertEquals(2, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isInfinite(lagrangeInequalityMultipliers[0]) || Double.isNaN(lagrangeInequalityMultipliers[0]));
      assertTrue(Double.isInfinite(lagrangeInequalityMultipliers[1]) || Double.isNaN(lagrangeInequalityMultipliers[1]));
      assertTrue(Double.isInfinite(lagrangeInequalityMultipliers[2]) || Double.isNaN(lagrangeInequalityMultipliers[2]));

      assertEquals(numberOfIterations, 1);
   }

   /**
    *  Test with dataset from sim that revealed a bug with the variable lower/upper bounds handling.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindValidSolutionForDataset20160319WithWarmStart()
   {
      ActualDatasetFrom20160319 dataset = new ActualDatasetFrom20160319();
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();
      solver.setUseWarmStart(true);

      solver.clear();
      solver.setQuadraticCostFunction(dataset.getCostQuadraticMatrix(), dataset.getCostLinearVector(), 0.0);
      solver.setVariableBounds(dataset.getVariableLowerBounds(), dataset.getVariableUpperBounds());
      DenseMatrix64F solution = new DenseMatrix64F(dataset.getProblemSize(), 1);
      solver.solve(solution);
      int numberOfIterations = solver.solve(solution);

      assertFalse(MatrixTools.containsNaN(solution));
      assertEquals(numberOfIterations, 1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindValidSolutionForKiwiDataset20170712WithWarmStart()
   {
      ActualDatasetFromKiwi20170712 dataset = new ActualDatasetFromKiwi20170712();
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();
      solver.setUseWarmStart(true);

      solver.clear();
      solver.setQuadraticCostFunction(dataset.getCostQuadraticMatrix(), dataset.getCostLinearVector(), 0.0);
      solver.setVariableBounds(dataset.getVariableLowerBounds(), dataset.getVariableUpperBounds());
      DenseMatrix64F solution = new DenseMatrix64F(dataset.getProblemSize(), 1);
      solver.solve(solution);
      int numberOfIterations = solver.solve(solution);

      assertFalse(MatrixTools.containsNaN(solution));
      assertEquals(numberOfIterations, 1);
   }

   /**
    * Test with dataset of a Kiwi simulation walking backward.
    * It seems that the problem is related to the fact that the robot has 6 contact points per foot.
    * The solver still fails when increasing the max number of iterations.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindValidSolutionForKiwiDataset20171013WithWarmStart()
   {
      ActualDatasetFromKiwi20171013 dataset = new ActualDatasetFromKiwi20171013();
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();
      solver.setUseWarmStart(true);

      solver.clear();
      solver.setQuadraticCostFunction(dataset.getCostQuadraticMatrix(), dataset.getCostLinearVector(), 0.0);
      solver.setVariableBounds(dataset.getVariableLowerBounds(), dataset.getVariableUpperBounds());
      DenseMatrix64F solution = new DenseMatrix64F(dataset.getProblemSize(), 1);
      solver.solve(solution);
      int numberOfIterations = solver.solve(solution);

      assertFalse(MatrixTools.containsNaN(solution));
      assertEquals(1, numberOfIterations);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindValidSolutionForKiwiDatasetProblemWithWarmStart()
   {
      ActualDatasetFromKiwi20171015A datasetA = new ActualDatasetFromKiwi20171015A();
      ActualDatasetFromKiwi20171015B datasetB = new ActualDatasetFromKiwi20171015B();
      DenseMatrix64F solution = new DenseMatrix64F(datasetA.getProblemSize(), 1);

      SimpleActiveSetQPSolverInterface solver = createSolverToTest();
      solver.setUseWarmStart(true);


      solver.clear();
      solver.setQuadraticCostFunction(datasetA.getCostQuadraticMatrix(), datasetA.getCostLinearVector(), 0.0);
      solver.setVariableBounds(datasetA.getVariableLowerBounds(), datasetA.getVariableUpperBounds());
      solver.solve(solution);

      solver.clear();
      solver.setQuadraticCostFunction(datasetA.getCostQuadraticMatrix(), datasetA.getCostLinearVector(), 0.0);
      solver.setVariableBounds(datasetA.getVariableLowerBounds(), datasetA.getVariableUpperBounds());
      solver.solve(solution);

      solver.clear();
      solver.setQuadraticCostFunction(datasetB.getCostQuadraticMatrix(), datasetB.getCostLinearVector(), 0.0);
      solver.setVariableBounds(datasetB.getVariableLowerBounds(), datasetB.getVariableUpperBounds());
      int numberOfIterationsWithWarmStart = solver.solve(solution);

      assertFalse(MatrixTools.containsNaN(solution));
      //assertEquals(numberOfIterationsWithWarmStart, 1);

      solver.setUseWarmStart(false);
      solver.clear();
      solver.setQuadraticCostFunction(datasetB.getCostQuadraticMatrix(), datasetB.getCostLinearVector(), 0.0);
      solver.setVariableBounds(datasetB.getVariableLowerBounds(), datasetB.getVariableUpperBounds());
      int numberOfIterationsWithoutWarmStart = solver.solve(solution);

      assertTrue(numberOfIterationsWithWarmStart < numberOfIterationsWithoutWarmStart);
   }
}
