package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ICPQPInputCalculatorTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testComputeQuadraticTask()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < 10; iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 1000);
         int subSize = RandomNumbers.nextInt(random, 1, size);
         int startIndex = RandomNumbers.nextInt(random, 1, (size - subSize));

         ICPQPInput inputToTest = new ICPQPInput(size);
         ICPQPInput inputExpected = new ICPQPInput(size);

         DenseMatrix64F weight = RandomMatrices.createRandom(subSize, subSize, random);
         DenseMatrix64F objective = RandomMatrices.createRandom(subSize, 1, random);

         ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
         ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
         inputCalculator.tmpObjective.reshape(subSize, 1);

         for (int row = 0; row < subSize; row++)
         {
            for (int col = 0; col < subSize; col++)
            {
               inputExpected.quadraticTerm.set(startIndex + row, startIndex + col, weight.get(row, col));
            }
         }

         DenseMatrix64F tempMatrix = new DenseMatrix64F(subSize, 1);
         DenseMatrix64F scalar = new DenseMatrix64F(1, 1);
         CommonOps.mult(weight, objective, tempMatrix);

         for (int row = 0; row < subSize; row++)
         {
            inputExpected.linearTerm.set(startIndex + row, 0, objective.get(row, 0));
         }

         inputCalculator.computeQuadraticTask(startIndex, inputToTest, weight, objective);

         CommonOps.multTransA(objective, tempMatrix, scalar);
         CommonOps.scale(0.5, scalar);
         inputExpected.residualCost.set(scalar);

         assertInputEquals(inputExpected, inputExpected, epsilon);
      }

      int size = RandomNumbers.nextInt(random, 1, 1000);
      int subSize = RandomNumbers.nextInt(random, 1, size);
      int startIndex = RandomNumbers.nextInt(random, 1, (size - subSize));

      ICPQPInput inputToTest = new ICPQPInput(size);
      ICPQPInput inputExpected = new ICPQPInput(size);

      DenseMatrix64F weight = RandomMatrices.createRandom(subSize, subSize, random);
      DenseMatrix64F objective = RandomMatrices.createRandom(subSize, 1, random);

      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
      inputCalculator.tmpObjective.reshape(subSize, 1);

      for (int iter = 0; iter < 10; iter++)
      {

         for (int row = 0; row < subSize; row++)
         {
            for (int col = 0; col < subSize; col++)
            {
               inputExpected.quadraticTerm.add(startIndex + row, startIndex + col, weight.get(row, col));
            }
         }

         DenseMatrix64F tempMatrix = new DenseMatrix64F(subSize, 1);
         DenseMatrix64F scalar = new DenseMatrix64F(1, 1);
         CommonOps.mult(weight, objective, tempMatrix);

         for (int row = 0; row < subSize; row++)
         {
            inputExpected.linearTerm.add(startIndex + row, 0, objective.get(row, 0));
         }

         inputCalculator.computeQuadraticTask(startIndex, inputToTest, weight, objective);

         CommonOps.multTransA(objective, tempMatrix, scalar);
         CommonOps.scale(0.5, scalar);
         inputExpected.residualCost.add(0, 0, scalar.get(0, 0));

         assertInputEquals(inputExpected, inputExpected, epsilon);
      }
   }

   @Test
   public void testFeedbackTask()
   {
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(2.0, feedbackWeight);

      CommonOps.setIdentity(icpQPInputExpected.quadraticTerm);
      CommonOps.scale(2.0, icpQPInputExpected.quadraticTerm);

      ICPQPInputCalculator.computeCoPFeedbackTask(icpQPInputToTest, feedbackWeight);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
   }

   @Test
   public void testCoPFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F rateWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(rateWeight);
      CommonOps.scale(2.0, rateWeight);

      DenseMatrix64F previousSolution = new DenseMatrix64F(2, 1);
      previousSolution.set(0, 0, 0.5);
      previousSolution.set(1, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(rateWeight);

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(rateWeight, previousSolution, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeCoPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 100; iter++)
      {
         icpQPInputToTest.reset();
         previousSolution.set(RandomMatrices.createRandom(2, 1, random));

         inputCalculator.computeCoPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

         CommonOps.mult(rateWeight, previousSolution, Qx_p);

         icpQPInputExpected.linearTerm.set(Qx_p);

         CommonOps.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
         CommonOps.scale(0.5, icpQPInputExpected.residualCost);

         assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
      }
   }

   @Test
   public void testCMPFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
      indexHandler.computeProblemSize();

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputEmpty = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(4);

      // test without cmp, which should be really easy
      DenseMatrix64F rateWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(rateWeight);
      CommonOps.scale(2.0, rateWeight);

      DenseMatrix64F previousSolution = new DenseMatrix64F(2, 1);
      previousSolution.set(0, 0, 0.5);
      previousSolution.set(1, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(2, 2, rateWeight.get(0, 0));
      icpQPInputExpected.quadraticTerm.set(3, 3, rateWeight.get(1, 1));

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(rateWeight, previousSolution, Qx_p);

      icpQPInputExpected.linearTerm.set(2, 0, Qx_p.get(0, 0));
      icpQPInputExpected.linearTerm.set(3, 0, Qx_p.get(1, 0));

      CommonOps.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeCMPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

      assertInputEquals(icpQPInputEmpty, icpQPInputToTest, epsilon);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      icpQPInputToTest = new ICPQPInput(4);

      inputCalculator.computeCMPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 100; iter++)
      {
         icpQPInputToTest.reset();
         previousSolution.set(RandomMatrices.createRandom(2, 1, random));

         inputCalculator.computeCMPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

         CommonOps.mult(rateWeight, previousSolution, Qx_p);

         icpQPInputExpected.linearTerm.set(2, 0, Qx_p.get(0, 0));
         icpQPInputExpected.linearTerm.set(3, 0, Qx_p.get(1, 0));

         CommonOps.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
         CommonOps.scale(0.5, icpQPInputExpected.residualCost);

         assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
      }
   }

   @Test
   public void testFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTestWithoutCMP = new ICPQPInput(2);
      ICPQPInput icpQPInputToTestWithCMP = new ICPQPInput(4);
      ICPQPInput icpQPInputWithoutCMP = new ICPQPInput(2);
      ICPQPInput icpQPInputWithCMP = new ICPQPInput(4);

      // test without cmp, which should be really easy
      DenseMatrix64F rateWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(rateWeight);
      CommonOps.scale(2.0, rateWeight);

      DenseMatrix64F previousCoPSolution = new DenseMatrix64F(2, 1);
      previousCoPSolution.set(0, 0, 0.5);
      previousCoPSolution.set(1, 0, 0.1);
      DenseMatrix64F previousCMPSolution = new DenseMatrix64F(2, 1);
      previousCMPSolution.set(0, 0, 0.3);
      previousCMPSolution.set(1, 0, 0.7);

      DenseMatrix64F previousSolution = new DenseMatrix64F(2, 1);
      CommonOps.add(previousCMPSolution, previousCoPSolution, previousSolution);

      icpQPInputWithoutCMP.quadraticTerm.set(rateWeight);
      MatrixTools.setDiagonal(icpQPInputWithCMP.quadraticTerm, 2.0);

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(rateWeight, previousCoPSolution, Qx_p);

      icpQPInputWithoutCMP.linearTerm.set(Qx_p);

      CommonOps.mult(rateWeight, previousSolution, Qx_p);

      icpQPInputWithCMP.linearTerm.set(0, 0, Qx_p.get(0, 0));
      icpQPInputWithCMP.linearTerm.set(1, 0, Qx_p.get(1, 0));
      icpQPInputWithCMP.linearTerm.set(2, 0, Qx_p.get(0, 0));
      icpQPInputWithCMP.linearTerm.set(3, 0, Qx_p.get(1, 0));

      CommonOps.mult(rateWeight, previousCoPSolution, Qx_p);

      CommonOps.multAddTransA(previousCoPSolution, Qx_p, icpQPInputWithoutCMP.residualCost);
      CommonOps.scale(0.5, icpQPInputWithoutCMP.residualCost);

      icpQPInputWithCMP.quadraticTerm.set(0, 2, 2.0);
      icpQPInputWithCMP.quadraticTerm.set(1, 3, 2.0);
      icpQPInputWithCMP.quadraticTerm.set(2, 0, 2.0);
      icpQPInputWithCMP.quadraticTerm.set(3, 1, 2.0);

      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithoutCMP, rateWeight, previousCoPSolution);

      assertInputEquals(icpQPInputWithoutCMP, icpQPInputToTestWithoutCMP, epsilon);

      CommonOps.mult(rateWeight, previousSolution, Qx_p);
      CommonOps.multAddTransA(previousSolution, Qx_p, icpQPInputWithCMP.residualCost);
      CommonOps.scale(0.5, icpQPInputWithCMP.residualCost);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithCMP, rateWeight, previousSolution);

      assertInputEquals(icpQPInputWithCMP, icpQPInputToTestWithCMP, epsilon);

      // run the actual test on the cost with zero previous solution.

      icpQPInputToTestWithCMP.reset();
      previousSolution.zero();
      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithCMP, rateWeight, previousSolution);

      DenseMatrix64F zeroLinear = new DenseMatrix64F(4, 1);
      JUnitTools.assertMatrixEquals(icpQPInputToTestWithCMP.linearTerm, zeroLinear, epsilon);
      zeroLinear.reshape(1, 1);
      JUnitTools.assertMatrixEquals(icpQPInputToTestWithCMP.residualCost, zeroLinear, epsilon);

      Random random = new Random(1738L);
      DenseMatrix64F solution = RandomMatrices.createRandom(4, 1, random);
      DenseMatrix64F compositeSolution = new DenseMatrix64F(2, 1);
      compositeSolution.set(0, 0, solution.get(0, 0) + solution.get(2, 0));
      compositeSolution.set(1, 0, solution.get(1, 0) + solution.get(3, 0));

      DenseMatrix64F tempMatrix = new DenseMatrix64F(2, 1);
      DenseMatrix64F expectedCost = new DenseMatrix64F(1, 1);

      CommonOps.mult(rateWeight, compositeSolution, tempMatrix);
      CommonOps.multTransA(compositeSolution, tempMatrix, expectedCost);
      CommonOps.scale(0.5, expectedCost);

      JUnitTools.assertMatrixEquals(expectedCost, computeCost(icpQPInputToTestWithCMP, solution), epsilon);
      Assert.assertEquals(expectedCost.get(0, 0), icpQPInputToTestWithCMP.computeCost(solution), epsilon);

      // run the actual test on the cost with non-zero previous solution.

      icpQPInputToTestWithCMP.reset();
      previousSolution.set(RandomMatrices.createRandom(4, 1, random));
      DenseMatrix64F compositePreviousSolution = new DenseMatrix64F(2, 1);
      compositePreviousSolution.set(0, 0, previousSolution.get(0, 0) + previousSolution.get(2, 0));
      compositePreviousSolution.set(1, 0, previousSolution.get(1, 0) + previousSolution.get(3, 0));

      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithCMP, rateWeight, compositePreviousSolution);

      solution = RandomMatrices.createRandom(4, 1, random);
      compositeSolution = new DenseMatrix64F(2, 1);
      compositeSolution.set(0, 0, solution.get(0, 0) + solution.get(2, 0));
      compositeSolution.set(1, 0, solution.get(1, 0) + solution.get(3, 0));

      tempMatrix = new DenseMatrix64F(2, 1);
      expectedCost = new DenseMatrix64F(1, 1);

      DenseMatrix64F delta = new DenseMatrix64F(4, 1);
      DenseMatrix64F compositeDelta = new DenseMatrix64F(2, 1);
      CommonOps.subtract(solution, previousSolution, delta);
      CommonOps.subtract(compositeSolution, compositePreviousSolution, compositeDelta);

      CommonOps.mult(rateWeight, compositeDelta, tempMatrix);
      CommonOps.multTransA(compositeDelta, tempMatrix, expectedCost);
      CommonOps.scale(0.5, expectedCost);

      JUnitTools.assertMatrixEquals(expectedCost, computeCost(icpQPInputToTestWithCMP, solution), epsilon);
      Assert.assertEquals(expectedCost.get(0, 0), icpQPInputToTestWithCMP.computeCost(solution), epsilon);
   }


   @Test
   public void testFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(footstepWeight);
      CommonOps.scale(2.0, footstepWeight);

      DenseMatrix64F footstepObjective = new DenseMatrix64F(2, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(0, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(footstepWeight);

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(footstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeFootstepTask(0, icpQPInputToTest, footstepWeight, footstepObjective);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective4 = new DenseMatrix64F(2, 1);
      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepObjective = new DenseMatrix64F(8, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(1, 0, 0.1);
      footstepObjective.set(2, 0, 1.0);
      footstepObjective.set(3, 0, -0.1);
      footstepObjective.set(4, 0, 1.5);
      footstepObjective.set(5, 0, 0.1);
      footstepObjective.set(6, 0, 2.0);
      footstepObjective.set(7, 0, -0.1);

      DenseMatrix64F bigFootstepWeight = new DenseMatrix64F(8, 8);
      CommonOps.setIdentity(bigFootstepWeight);
      CommonOps.scale(2.0, bigFootstepWeight);

      icpQPInputExpected.reshape(8);
      icpQPInputExpected.reset();
      icpQPInputToTest.reshape(8);
      icpQPInputToTest.reset();

      inputCalculator.computeFootstepTask(0, icpQPInputToTest, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepTask(1, icpQPInputToTest, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepTask(2, icpQPInputToTest, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepTask(3, icpQPInputToTest, footstepWeight, footstepObjective4);

      icpQPInputExpected.quadraticTerm.set(bigFootstepWeight);

      Qx_p = new DenseMatrix64F(8, 1);
      CommonOps.mult(bigFootstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
   }

   @Test
   public void testFootstepRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(footstepWeight);
      CommonOps.scale(2.0, footstepWeight);

      DenseMatrix64F footstepObjective = new DenseMatrix64F(2, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(0, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(footstepWeight);

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(footstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeFootstepRateTask(0, icpQPInputToTest, footstepWeight, footstepObjective);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      // test multiple footsteps
      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective4 = new DenseMatrix64F(2, 1);
      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepObjective = new DenseMatrix64F(8, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(1, 0, 0.1);
      footstepObjective.set(2, 0, 1.0);
      footstepObjective.set(3, 0, -0.1);
      footstepObjective.set(4, 0, 1.5);
      footstepObjective.set(5, 0, 0.1);
      footstepObjective.set(6, 0, 2.0);
      footstepObjective.set(7, 0, -0.1);

      DenseMatrix64F bigFootstepWeight = new DenseMatrix64F(8, 8);
      CommonOps.setIdentity(bigFootstepWeight);
      CommonOps.scale(2.0, bigFootstepWeight);

      icpQPInputExpected.reshape(8);
      icpQPInputExpected.reset();
      icpQPInputToTest.reshape(8);
      icpQPInputToTest.reset();

      inputCalculator.computeFootstepRateTask(0, icpQPInputToTest, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(1, icpQPInputToTest, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepRateTask(2, icpQPInputToTest, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepRateTask(3, icpQPInputToTest, footstepWeight, footstepObjective4);

      icpQPInputExpected.quadraticTerm.set(bigFootstepWeight);

      Qx_p = new DenseMatrix64F(8, 1);
      CommonOps.mult(bigFootstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithFeedbackAndAngularMomentum()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGain);
      CommonOps.scale(gain, feedbackGain);

      DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(invertedFeedbackGain);
      CommonOps.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DenseMatrix64F weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      // test just feedback
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(2);
      icpQPInputToTest.reshape(2);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      DenseMatrix64F tmpMatrix = new DenseMatrix64F(2, 2);
      CommonOps.mult(weightMatrix, invertedFeedbackGain, tmpMatrix);
      CommonOps.mult(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.quadraticTerm);

      tmpMatrix = new DenseMatrix64F(2, 1);
      CommonOps.mult(weightMatrix, currentICPError, tmpMatrix);
      CommonOps.multTransA(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.linearTerm);
      CommonOps.multTransA(0.5, currentICPError, tmpMatrix, icpQPInputExpected.residualCost);

      DenseMatrix64F feedbackJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      DenseMatrix64F adjustmentJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);
      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and step adjustment
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, feedbackObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum and step adjustment
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 6);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 6);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 4, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, feedbackObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(6, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithFeedback()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGain);
      CommonOps.scale(gain, feedbackGain);

      DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(invertedFeedbackGain);
      CommonOps.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DenseMatrix64F weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      // test just feedback
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(2);
      icpQPInputToTest.reshape(2);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      DenseMatrix64F feedbackJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      DenseMatrix64F adjustmentJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);
      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);

      DenseMatrix64F tmpMatrix = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // ONLY THE FEEDBACK ONE IS ENOUGH
      // test feedback and step adjustment
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, feedbackObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // ONLY THE FEEDBACK ONE IS ENOUGH
      // test feedback and angular momentum
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();
      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum and step adjustment
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();
      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 6);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 6);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 4, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(6, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithAngularMomentum()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGain);
      CommonOps.scale(gain, feedbackGain);

      DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(invertedFeedbackGain);
      CommonOps.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DenseMatrix64F weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      // test just feedback
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reshape(2);
      icpQPInputToTest.reshape(2);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      DenseMatrix64F feedbackJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      DenseMatrix64F adjustmentJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);
      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);

      DenseMatrix64F tmpMatrix = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and step adjustment
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 2, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum and step adjustment
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 6);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 6);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 4, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(6, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithSeparateAdjustment()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGain);
      CommonOps.scale(gain, feedbackGain);

      DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(invertedFeedbackGain);
      CommonOps.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DenseMatrix64F weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F feedbackJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F feedbackObjectiveExpected = new DenseMatrix64F(2, 1);

      DenseMatrix64F adjustmentJacobianExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      // test just feedback
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reshape(2);
      icpQPInputToTest.reshape(2);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      DenseMatrix64F tmpMatrix = new DenseMatrix64F(2, 2);
      CommonOps.mult(weightMatrix, invertedFeedbackGain, tmpMatrix);
      CommonOps.mult(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.quadraticTerm);

      tmpMatrix = new DenseMatrix64F(2, 1);
      CommonOps.mult(weightMatrix, currentICPError, tmpMatrix);
      CommonOps.multTransA(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.linearTerm);
      CommonOps.multTransA(0.5, currentICPError, tmpMatrix, icpQPInputExpected.residualCost);

      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 2);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and step adjustment
      indexHandler.setHasCMPFeedbackTask(false);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 2, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 4);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 4);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

      // test feedback and angular momentum and step adjustment
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.resetFootsteps();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      inputCalculator
            .computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      feedbackJacobianExpected = new DenseMatrix64F(2, 6);
      feedbackObjectiveExpected = new DenseMatrix64F(2, 1);
      adjustmentJacobianExpected = new DenseMatrix64F(2, 6);
      adjustmentObjectiveExpected = new DenseMatrix64F(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 4, CommonOps.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DenseMatrix64F(6, 2);
      CommonOps.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      JUnitTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      JUnitTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      JUnitTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      JUnitTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      JUnitTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testSubmitCoPeedbackTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);
      ICPQPInput feedbackRateTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      DenseMatrix64F feedbackRateWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackRateWeight);
      CommonOps.scale(1.3, feedbackRateWeight);

      DenseMatrix64F previousFeedbackSolution = new DenseMatrix64F(2, 1);
      previousFeedbackSolution.set(0, 0, 0.03);
      previousFeedbackSolution.set(1, 0, 0.05);

      inputCalculator.computeCoPFeedbackRateTask(feedbackRateTask, feedbackRateWeight, previousFeedbackSolution);

      DenseMatrix64F quadratic = new DenseMatrix64F(2, 2);
      DenseMatrix64F linear = new DenseMatrix64F(2, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F linearExpected = new DenseMatrix64F(2, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackRateTask, quadratic, linear, scalar);

      quadraticExpected.set(feedbackTask.quadraticTerm);
      linearExpected.set(feedbackTask.linearTerm);
      scalarExpected.set(feedbackTask.residualCost);

      CommonOps.addEquals(quadraticExpected, feedbackRateTask.quadraticTerm);
      CommonOps.addEquals(linearExpected, feedbackRateTask.linearTerm);
      CommonOps.addEquals(scalarExpected, feedbackRateTask.residualCost);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);


   }

   @Test
   public void testSubmitFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackRateTask = new ICPQPInput(4);

      DenseMatrix64F feedbackRateWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F feedbackCMPCoPRateWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackRateWeight);
      CommonOps.scale(1.3, feedbackRateWeight);
      MatrixTools.setDiagonal(feedbackCMPCoPRateWeight, 4.7);

      DenseMatrix64F previousCoPFeedbackSolution = new DenseMatrix64F(2, 1);
      DenseMatrix64F previousCMPFeedbackSolution = new DenseMatrix64F(2, 1);
      DenseMatrix64F previousFeedbackSolution = new DenseMatrix64F(2, 1);
      previousCoPFeedbackSolution.set(0, 0, 0.06);
      previousCoPFeedbackSolution.set(1, 0, 0.13);
      previousCMPFeedbackSolution.set(0, 0, 0.03);
      previousCMPFeedbackSolution.set(1, 0, 0.07);

      CommonOps.add(previousCoPFeedbackSolution, previousCMPFeedbackSolution, previousFeedbackSolution);

      inputCalculator.computeCoPFeedbackRateTask(feedbackRateTask, feedbackCMPCoPRateWeight, previousCoPFeedbackSolution);
      inputCalculator.computeCMPFeedbackRateTask(feedbackRateTask, feedbackCMPCoPRateWeight, previousCMPFeedbackSolution);
      inputCalculator.computeFeedbackRateTask(feedbackRateTask, feedbackRateWeight, previousFeedbackSolution);

      DenseMatrix64F quadratic = new DenseMatrix64F(4, 4);
      DenseMatrix64F linear = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(4, 4);
      DenseMatrix64F linearExpected = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFeedbackRateTask(feedbackRateTask, quadratic, linear, scalar);

      quadraticExpected.set(feedbackRateTask.quadraticTerm);
      linearExpected.set(feedbackRateTask.linearTerm);
      scalarExpected.set(feedbackRateTask.residualCost);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, epsilon);
      JUnitTools.assertMatrixEquals(linearExpected, linear, epsilon);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, epsilon);

      quadraticExpected = new DenseMatrix64F(4, 4);

      quadraticExpected.set(0, 0, feedbackCMPCoPRateWeight.get(0, 0) + feedbackRateWeight.get(0, 0));
      quadraticExpected.set(1, 1, feedbackCMPCoPRateWeight.get(1, 1) + feedbackRateWeight.get(1, 1));
      quadraticExpected.set(2, 2, feedbackCMPCoPRateWeight.get(0, 0) + feedbackRateWeight.get(0, 0));
      quadraticExpected.set(3, 3, feedbackCMPCoPRateWeight.get(1, 1) + feedbackRateWeight.get(1, 1));
      quadraticExpected.set(0, 2, feedbackRateWeight.get(0, 0));
      quadraticExpected.set(1, 3, feedbackRateWeight.get(1, 1));
      quadraticExpected.set(2, 0, feedbackRateWeight.get(0, 0));
      quadraticExpected.set(3, 1, feedbackRateWeight.get(1, 1));

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, epsilon);

   }

   @Test
   public void testSubmitDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(2.7, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);

      quadraticExpected.set(dynamicsTask.quadraticTerm);
      linearExpected.set(dynamicsTask.linearTerm);
      scalarExpected.set(dynamicsTask.residualCost);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitCMPFeedbackTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DenseMatrix64F angularMomentumWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(angularMomentumWeight);
      CommonOps.scale(2.0, angularMomentumWeight);

      inputCalculator.computeCMPFeedbackTask(angularMomentumTask, angularMomentumWeight);

      DenseMatrix64F quadratic = new DenseMatrix64F(4, 4);
      DenseMatrix64F linear = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(4, 4);
      DenseMatrix64F linearExpected = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitCMPFeedbackTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      CommonOps.addEquals(scalarExpected, angularMomentumTask.residualCost);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      ICPQPInput footstepTask = new ICPQPInput(8);
      ICPQPInput footstepRateTask = new ICPQPInput(8);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective4 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious4 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);
      footstepPrevious2.set(0, 0, 1.02);
      footstepPrevious2.set(1, 0, -0.08);
      footstepPrevious3.set(0, 0, 1.51);
      footstepPrevious3.set(1, 0, 0.095);
      footstepPrevious4.set(0, 0, 2.03);
      footstepPrevious4.set(1, 0, -0.14);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRateWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepTask(1, footstepTask, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepTask(2, footstepTask, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepTask(3, footstepTask, footstepWeight, footstepObjective4);

      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);
      inputCalculator.computeFootstepRateTask(1, footstepRateTask, footstepRateWeight, footstepPrevious2);
      inputCalculator.computeFootstepRateTask(2, footstepRateTask, footstepRateWeight, footstepPrevious3);
      inputCalculator.computeFootstepRateTask(3, footstepRateTask, footstepRateWeight, footstepPrevious4);

      DenseMatrix64F quadratic = new DenseMatrix64F(10, 10);
      DenseMatrix64F linear = new DenseMatrix64F(10, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(10, 10);
      DenseMatrix64F linearExpected = new DenseMatrix64F(10, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRateTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 2, 2, footstepTask.quadraticTerm, 0, 0, 8, 8, 1.0);
      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, footstepRateTask.quadraticTerm, 0, 0, 8, 8, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 2, 0, footstepTask.linearTerm, 0, 0, 8, 1, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, footstepRateTask.linearTerm, 0, 0, 8, 1, 1.0);
      scalarExpected.set(footstepTask.residualCost);
      CommonOps.addEquals(scalarExpected, footstepRateTask.residualCost);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitCoPAndCMPOFeedbackTasks()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DenseMatrix64F angularMomentumWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(angularMomentumWeight);
      CommonOps.scale(2.0, angularMomentumWeight);

      inputCalculator.computeCMPFeedbackTask(angularMomentumTask, angularMomentumWeight);

      DenseMatrix64F quadratic = new DenseMatrix64F(4, 4);
      DenseMatrix64F linear = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(4, 4);
      DenseMatrix64F linearExpected = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitCMPFeedbackTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, angularMomentumTask.residualCost, 0, 0, 1, 1, 1.0);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.registerFootstep();
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRateTask = new ICPQPInput(2);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRateWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRateTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepRateTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepRateTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepRateTask.residualCost, 0, 0, 1, 1, 1.0);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndFootstepAndDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRateTask = new ICPQPInput(2);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRateWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);
      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRateTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepRateTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepRateTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepRateTask.residualCost, 0, 0, 1, 1, 1.0);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      JUnitTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndFootstepAndDynamicsAndAngularMomentumTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRateTask = new ICPQPInput(2);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRateWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DenseMatrix64F angularMomentumWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(angularMomentumWeight);
      CommonOps.scale(2.0, angularMomentumWeight);

      inputCalculator.computeCMPFeedbackTask(angularMomentumTask, angularMomentumWeight);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRateTask, quadratic, linear, scalar);
      inputCalculator.submitCMPFeedbackTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepRateTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepRateTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepRateTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, angularMomentumTask.residualCost, 0, 0, 1, 1, 1.0);

      JUnitTools.assertMatrixEquals(quadraticExpected, quadratic, epsilon);
      JUnitTools.assertMatrixEquals(linearExpected, linear, epsilon);
      JUnitTools.assertMatrixEquals(scalarExpected, scalar, epsilon);
   }

   @Test
   public void testComputeDynamicsConstraintError()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoVariableRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
      Random random = new Random(1738L);

      for (int iter = 0; iter < 100; iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 100);
         DenseMatrix64F solution = new DenseMatrix64F(size, 1);

         DenseMatrix64F feedbackJacobian = RandomMatrices.createRandom(2, size, random);
         DenseMatrix64F adjustmentJacobian = RandomMatrices.createRandom(2, size, random);

         DenseMatrix64F feedbackObjective = RandomMatrices.createRandom(2, 1, random);
         DenseMatrix64F adjustmentObjective = RandomMatrices.createRandom(2, 1, random);

         inputCalculator.feedbackJacobian.set(feedbackJacobian);
         inputCalculator.adjustmentJacobian.set(adjustmentJacobian);

         inputCalculator.feedbackObjective.set(feedbackObjective);
         inputCalculator.adjustmentObjective.set(adjustmentObjective);

         DenseMatrix64F errorToTest = new DenseMatrix64F(2, 1);
         DenseMatrix64F errorExpected = new DenseMatrix64F(2, 1);

         CommonOps.multAdd(feedbackJacobian, solution, errorExpected);
         CommonOps.multAdd(adjustmentJacobian, solution, errorExpected);
         CommonOps.addEquals(errorExpected, -1.0, feedbackObjective);
         CommonOps.addEquals(errorExpected, -1.0, adjustmentObjective);

         inputCalculator.computeDynamicConstraintError(solution, errorToTest);

         JUnitTools.assertMatrixEquals(errorExpected, errorToTest, epsilon);
      }

   }

   private static void assertInputEquals(ICPQPInput inputA, ICPQPInput inputB, double tol)
   {
      JUnitTools.assertMatrixEquals("Quadratic terms aren't equal.", inputA.quadraticTerm, inputB.quadraticTerm, tol);
      JUnitTools.assertMatrixEquals("Linear terms aren't equal.", inputA.linearTerm, inputB.linearTerm, tol);
      JUnitTools.assertMatrixEquals("Residual terms aren't equal.", inputA.residualCost, inputB.residualCost, tol);
   }


   private static DenseMatrix64F computeCost(ICPQPInput icpqpInput, DenseMatrix64F solution)
   {
      DenseMatrix64F tempMatrix = new DenseMatrix64F(solution.numRows, 1);
      DenseMatrix64F cost = new DenseMatrix64F(1, 1);

      CommonOps.mult(icpqpInput.quadraticTerm, solution, tempMatrix);
      CommonOps.multTransA(solution, tempMatrix, cost);
      CommonOps.scale(0.5, cost);

      CommonOps.multAddTransA(-1.0, icpqpInput.linearTerm, solution, cost);
      CommonOps.addEquals(cost, icpqpInput.residualCost);

      return cost;
   }
}
