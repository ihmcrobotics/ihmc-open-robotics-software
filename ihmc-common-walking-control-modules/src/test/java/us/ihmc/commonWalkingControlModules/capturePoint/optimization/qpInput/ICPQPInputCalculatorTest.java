package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoRegistry;

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

         DMatrixRMaj weight = RandomMatrices_DDRM.rectangle(subSize, subSize, random);
         DMatrixRMaj objective = RandomMatrices_DDRM.rectangle(subSize, 1, random);

         ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
         ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
         inputCalculator.tmpObjective.reshape(subSize, 1);

         for (int row = 0; row < subSize; row++)
         {
            for (int col = 0; col < subSize; col++)
            {
               inputExpected.quadraticTerm.set(startIndex + row, startIndex + col, weight.get(row, col));
            }
         }

         DMatrixRMaj tempMatrix = new DMatrixRMaj(subSize, 1);
         DMatrixRMaj scalar = new DMatrixRMaj(1, 1);
         CommonOps_DDRM.mult(weight, objective, tempMatrix);

         for (int row = 0; row < subSize; row++)
         {
            inputExpected.linearTerm.set(startIndex + row, 0, objective.get(row, 0));
         }

         inputCalculator.computeQuadraticTask(startIndex, inputToTest, weight, objective);

         CommonOps_DDRM.multTransA(objective, tempMatrix, scalar);
         CommonOps_DDRM.scale(0.5, scalar);
         inputExpected.residualCost.set(scalar);

         assertInputEquals(inputExpected, inputExpected, epsilon);
      }

      int size = RandomNumbers.nextInt(random, 1, 1000);
      int subSize = RandomNumbers.nextInt(random, 1, size);
      int startIndex = RandomNumbers.nextInt(random, 1, (size - subSize));

      ICPQPInput inputToTest = new ICPQPInput(size);
      ICPQPInput inputExpected = new ICPQPInput(size);

      DMatrixRMaj weight = RandomMatrices_DDRM.rectangle(subSize, subSize, random);
      DMatrixRMaj objective = RandomMatrices_DDRM.rectangle(subSize, 1, random);

      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
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

         DMatrixRMaj tempMatrix = new DMatrixRMaj(subSize, 1);
         DMatrixRMaj scalar = new DMatrixRMaj(1, 1);
         CommonOps_DDRM.mult(weight, objective, tempMatrix);

         for (int row = 0; row < subSize; row++)
         {
            inputExpected.linearTerm.add(startIndex + row, 0, objective.get(row, 0));
         }

         inputCalculator.computeQuadraticTask(startIndex, inputToTest, weight, objective);

         CommonOps_DDRM.multTransA(objective, tempMatrix, scalar);
         CommonOps_DDRM.scale(0.5, scalar);
         inputExpected.residualCost.add(0, 0, scalar.get(0, 0));

         assertInputEquals(inputExpected, inputExpected, epsilon);
      }
   }

   @Test
   public void testFeedbackTask()
   {
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(2.0, feedbackWeight);

      CommonOps_DDRM.setIdentity(icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.scale(2.0, icpQPInputExpected.quadraticTerm);

      ICPQPInputCalculator.computeCoPFeedbackTask(icpQPInputToTest, feedbackWeight);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
   }

   @Test
   public void testCoPFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DMatrixRMaj rateWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(rateWeight);
      CommonOps_DDRM.scale(2.0, rateWeight);

      DMatrixRMaj previousSolution = new DMatrixRMaj(2, 1);
      previousSolution.set(0, 0, 0.5);
      previousSolution.set(1, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(rateWeight);

      DMatrixRMaj Qx_p = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(rateWeight, previousSolution, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps_DDRM.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeCoPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 100; iter++)
      {
         icpQPInputToTest.reset();
         previousSolution.set(RandomMatrices_DDRM.rectangle(2, 1, random));

         inputCalculator.computeCoPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

         CommonOps_DDRM.mult(rateWeight, previousSolution, Qx_p);

         icpQPInputExpected.linearTerm.set(Qx_p);

         CommonOps_DDRM.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
         CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

         assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
      }
   }

   @Test
   public void testCMPFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
      indexHandler.computeProblemSize();

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputEmpty = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(4);

      // test without cmp, which should be really easy
      DMatrixRMaj rateWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(rateWeight);
      CommonOps_DDRM.scale(2.0, rateWeight);

      DMatrixRMaj previousSolution = new DMatrixRMaj(2, 1);
      previousSolution.set(0, 0, 0.5);
      previousSolution.set(1, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(2, 2, rateWeight.get(0, 0));
      icpQPInputExpected.quadraticTerm.set(3, 3, rateWeight.get(1, 1));

      DMatrixRMaj Qx_p = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(rateWeight, previousSolution, Qx_p);

      icpQPInputExpected.linearTerm.set(2, 0, Qx_p.get(0, 0));
      icpQPInputExpected.linearTerm.set(3, 0, Qx_p.get(1, 0));

      CommonOps_DDRM.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

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
         previousSolution.set(RandomMatrices_DDRM.rectangle(2, 1, random));

         inputCalculator.computeCMPFeedbackRateTask(icpQPInputToTest, rateWeight, previousSolution);

         CommonOps_DDRM.mult(rateWeight, previousSolution, Qx_p);

         icpQPInputExpected.linearTerm.set(2, 0, Qx_p.get(0, 0));
         icpQPInputExpected.linearTerm.set(3, 0, Qx_p.get(1, 0));

         CommonOps_DDRM.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
         CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

         assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
      }
   }

   @Test
   public void testFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTestWithoutCMP = new ICPQPInput(2);
      ICPQPInput icpQPInputToTestWithCMP = new ICPQPInput(4);
      ICPQPInput icpQPInputWithoutCMP = new ICPQPInput(2);
      ICPQPInput icpQPInputWithCMP = new ICPQPInput(4);

      // test without cmp, which should be really easy
      DMatrixRMaj rateWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(rateWeight);
      CommonOps_DDRM.scale(2.0, rateWeight);

      DMatrixRMaj previousCoPSolution = new DMatrixRMaj(2, 1);
      previousCoPSolution.set(0, 0, 0.5);
      previousCoPSolution.set(1, 0, 0.1);
      DMatrixRMaj previousCMPSolution = new DMatrixRMaj(2, 1);
      previousCMPSolution.set(0, 0, 0.3);
      previousCMPSolution.set(1, 0, 0.7);

      DMatrixRMaj previousSolution = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.add(previousCMPSolution, previousCoPSolution, previousSolution);

      icpQPInputWithoutCMP.quadraticTerm.set(rateWeight);
      MatrixTools.setDiagonal(icpQPInputWithCMP.quadraticTerm, 2.0);

      DMatrixRMaj Qx_p = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(rateWeight, previousCoPSolution, Qx_p);

      icpQPInputWithoutCMP.linearTerm.set(Qx_p);

      CommonOps_DDRM.mult(rateWeight, previousSolution, Qx_p);

      icpQPInputWithCMP.linearTerm.set(0, 0, Qx_p.get(0, 0));
      icpQPInputWithCMP.linearTerm.set(1, 0, Qx_p.get(1, 0));
      icpQPInputWithCMP.linearTerm.set(2, 0, Qx_p.get(0, 0));
      icpQPInputWithCMP.linearTerm.set(3, 0, Qx_p.get(1, 0));

      CommonOps_DDRM.mult(rateWeight, previousCoPSolution, Qx_p);

      CommonOps_DDRM.multAddTransA(previousCoPSolution, Qx_p, icpQPInputWithoutCMP.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputWithoutCMP.residualCost);

      icpQPInputWithCMP.quadraticTerm.set(0, 2, 2.0);
      icpQPInputWithCMP.quadraticTerm.set(1, 3, 2.0);
      icpQPInputWithCMP.quadraticTerm.set(2, 0, 2.0);
      icpQPInputWithCMP.quadraticTerm.set(3, 1, 2.0);

      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithoutCMP, rateWeight, previousCoPSolution);

      assertInputEquals(icpQPInputWithoutCMP, icpQPInputToTestWithoutCMP, epsilon);

      CommonOps_DDRM.mult(rateWeight, previousSolution, Qx_p);
      CommonOps_DDRM.multAddTransA(previousSolution, Qx_p, icpQPInputWithCMP.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputWithCMP.residualCost);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithCMP, rateWeight, previousSolution);

      assertInputEquals(icpQPInputWithCMP, icpQPInputToTestWithCMP, epsilon);

      // run the actual test on the cost with zero previous solution.

      icpQPInputToTestWithCMP.reset();
      previousSolution.zero();
      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithCMP, rateWeight, previousSolution);

      DMatrixRMaj zeroLinear = new DMatrixRMaj(4, 1);
      MatrixTestTools.assertMatrixEquals(icpQPInputToTestWithCMP.linearTerm, zeroLinear, epsilon);
      zeroLinear.reshape(1, 1);
      MatrixTestTools.assertMatrixEquals(icpQPInputToTestWithCMP.residualCost, zeroLinear, epsilon);

      Random random = new Random(1738L);
      DMatrixRMaj solution = RandomMatrices_DDRM.rectangle(4, 1, random);
      DMatrixRMaj compositeSolution = new DMatrixRMaj(2, 1);
      compositeSolution.set(0, 0, solution.get(0, 0) + solution.get(2, 0));
      compositeSolution.set(1, 0, solution.get(1, 0) + solution.get(3, 0));

      DMatrixRMaj tempMatrix = new DMatrixRMaj(2, 1);
      DMatrixRMaj expectedCost = new DMatrixRMaj(1, 1);

      CommonOps_DDRM.mult(rateWeight, compositeSolution, tempMatrix);
      CommonOps_DDRM.multTransA(compositeSolution, tempMatrix, expectedCost);
      CommonOps_DDRM.scale(0.5, expectedCost);

      MatrixTestTools.assertMatrixEquals(expectedCost, computeCost(icpQPInputToTestWithCMP, solution), epsilon);
      Assert.assertEquals(expectedCost.get(0, 0), icpQPInputToTestWithCMP.computeCost(solution), epsilon);

      // run the actual test on the cost with non-zero previous solution.

      icpQPInputToTestWithCMP.reset();
      previousSolution.set(RandomMatrices_DDRM.rectangle(4, 1, random));
      DMatrixRMaj compositePreviousSolution = new DMatrixRMaj(2, 1);
      compositePreviousSolution.set(0, 0, previousSolution.get(0, 0) + previousSolution.get(2, 0));
      compositePreviousSolution.set(1, 0, previousSolution.get(1, 0) + previousSolution.get(3, 0));

      inputCalculator.computeFeedbackRateTask(icpQPInputToTestWithCMP, rateWeight, compositePreviousSolution);

      solution = RandomMatrices_DDRM.rectangle(4, 1, random);
      compositeSolution = new DMatrixRMaj(2, 1);
      compositeSolution.set(0, 0, solution.get(0, 0) + solution.get(2, 0));
      compositeSolution.set(1, 0, solution.get(1, 0) + solution.get(3, 0));

      tempMatrix = new DMatrixRMaj(2, 1);
      expectedCost = new DMatrixRMaj(1, 1);

      DMatrixRMaj delta = new DMatrixRMaj(4, 1);
      DMatrixRMaj compositeDelta = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.subtract(solution, previousSolution, delta);
      CommonOps_DDRM.subtract(compositeSolution, compositePreviousSolution, compositeDelta);

      CommonOps_DDRM.mult(rateWeight, compositeDelta, tempMatrix);
      CommonOps_DDRM.multTransA(compositeDelta, tempMatrix, expectedCost);
      CommonOps_DDRM.scale(0.5, expectedCost);

      MatrixTestTools.assertMatrixEquals(expectedCost, computeCost(icpQPInputToTestWithCMP, solution), epsilon);
      Assert.assertEquals(expectedCost.get(0, 0), icpQPInputToTestWithCMP.computeCost(solution), epsilon);
   }


   @Test
   public void testFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DMatrixRMaj footstepWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(footstepWeight);
      CommonOps_DDRM.scale(2.0, footstepWeight);

      DMatrixRMaj footstepObjective = new DMatrixRMaj(2, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(0, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(footstepWeight);

      DMatrixRMaj Qx_p = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(footstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps_DDRM.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeFootstepTask(0, icpQPInputToTest, footstepWeight, footstepObjective);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      DMatrixRMaj footstepObjective1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective2 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective3 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective4 = new DMatrixRMaj(2, 1);
      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepObjective = new DMatrixRMaj(8, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(1, 0, 0.1);
      footstepObjective.set(2, 0, 1.0);
      footstepObjective.set(3, 0, -0.1);
      footstepObjective.set(4, 0, 1.5);
      footstepObjective.set(5, 0, 0.1);
      footstepObjective.set(6, 0, 2.0);
      footstepObjective.set(7, 0, -0.1);

      DMatrixRMaj bigFootstepWeight = new DMatrixRMaj(8, 8);
      CommonOps_DDRM.setIdentity(bigFootstepWeight);
      CommonOps_DDRM.scale(2.0, bigFootstepWeight);

      icpQPInputExpected.reshape(8);
      icpQPInputExpected.reset();
      icpQPInputToTest.reshape(8);
      icpQPInputToTest.reset();

      inputCalculator.computeFootstepTask(0, icpQPInputToTest, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepTask(1, icpQPInputToTest, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepTask(2, icpQPInputToTest, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepTask(3, icpQPInputToTest, footstepWeight, footstepObjective4);

      icpQPInputExpected.quadraticTerm.set(bigFootstepWeight);

      Qx_p = new DMatrixRMaj(8, 1);
      CommonOps_DDRM.mult(bigFootstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps_DDRM.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
   }

   @Test
   public void testFootstepRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DMatrixRMaj footstepWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(footstepWeight);
      CommonOps_DDRM.scale(2.0, footstepWeight);

      DMatrixRMaj footstepObjective = new DMatrixRMaj(2, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(0, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(footstepWeight);

      DMatrixRMaj Qx_p = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(footstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps_DDRM.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeFootstepRateTask(0, icpQPInputToTest, footstepWeight, footstepObjective);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);

      // test multiple footsteps
      DMatrixRMaj footstepObjective1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective2 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective3 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective4 = new DMatrixRMaj(2, 1);
      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepObjective = new DMatrixRMaj(8, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(1, 0, 0.1);
      footstepObjective.set(2, 0, 1.0);
      footstepObjective.set(3, 0, -0.1);
      footstepObjective.set(4, 0, 1.5);
      footstepObjective.set(5, 0, 0.1);
      footstepObjective.set(6, 0, 2.0);
      footstepObjective.set(7, 0, -0.1);

      DMatrixRMaj bigFootstepWeight = new DMatrixRMaj(8, 8);
      CommonOps_DDRM.setIdentity(bigFootstepWeight);
      CommonOps_DDRM.scale(2.0, bigFootstepWeight);

      icpQPInputExpected.reshape(8);
      icpQPInputExpected.reset();
      icpQPInputToTest.reshape(8);
      icpQPInputToTest.reset();

      inputCalculator.computeFootstepRateTask(0, icpQPInputToTest, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(1, icpQPInputToTest, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepRateTask(2, icpQPInputToTest, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepRateTask(3, icpQPInputToTest, footstepWeight, footstepObjective4);

      icpQPInputExpected.quadraticTerm.set(bigFootstepWeight);

      Qx_p = new DMatrixRMaj(8, 1);
      CommonOps_DDRM.mult(bigFootstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps_DDRM.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps_DDRM.scale(0.5, icpQPInputExpected.residualCost);

      assertInputEquals(icpQPInputExpected, icpQPInputToTest, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithFeedbackAndAngularMomentum()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DMatrixRMaj feedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGain);
      CommonOps_DDRM.scale(gain, feedbackGain);

      DMatrixRMaj invertedFeedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(invertedFeedbackGain);
      CommonOps_DDRM.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DMatrixRMaj weightMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(weightMatrix);
      CommonOps_DDRM.scale(weight, weightMatrix);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
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

      DMatrixRMaj tmpMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.mult(weightMatrix, invertedFeedbackGain, tmpMatrix);
      CommonOps_DDRM.mult(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.quadraticTerm);

      tmpMatrix = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(weightMatrix, currentICPError, tmpMatrix);
      CommonOps_DDRM.multTransA(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.linearTerm);
      CommonOps_DDRM.multTransA(0.5, currentICPError, tmpMatrix, icpQPInputExpected.residualCost);

      DMatrixRMaj feedbackJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      DMatrixRMaj adjustmentJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);
      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, feedbackObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 6);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 6);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 4, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, feedbackObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(6, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithFeedback()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DMatrixRMaj feedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGain);
      CommonOps_DDRM.scale(gain, feedbackGain);

      DMatrixRMaj invertedFeedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(invertedFeedbackGain);
      CommonOps_DDRM.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DMatrixRMaj weightMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(weightMatrix);
      CommonOps_DDRM.scale(weight, weightMatrix);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
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

      DMatrixRMaj feedbackJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      DMatrixRMaj adjustmentJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);
      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);

      DMatrixRMaj tmpMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, feedbackObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 6);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 6);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 4, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(6, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps_DDRM.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps_DDRM.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithAngularMomentum()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DMatrixRMaj feedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGain);
      CommonOps_DDRM.scale(gain, feedbackGain);

      DMatrixRMaj invertedFeedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(invertedFeedbackGain);
      CommonOps_DDRM.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DMatrixRMaj weightMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(weightMatrix);
      CommonOps_DDRM.scale(weight, weightMatrix);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
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

      DMatrixRMaj feedbackJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      DMatrixRMaj adjustmentJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);
      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);

      DMatrixRMaj tmpMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);

      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 2, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps_DDRM.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps_DDRM.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 6);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 6);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 4, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(6, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps_DDRM.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps_DDRM.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testComputeDynamicsTaskWithSeparateAdjustment()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DMatrixRMaj feedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGain);
      CommonOps_DDRM.scale(gain, feedbackGain);

      DMatrixRMaj invertedFeedbackGain = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(invertedFeedbackGain);
      CommonOps_DDRM.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DMatrixRMaj weightMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(weightMatrix);
      CommonOps_DDRM.scale(weight, weightMatrix);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DMatrixRMaj feedbackJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj feedbackObjectiveExpected = new DMatrixRMaj(2, 1);

      DMatrixRMaj adjustmentJacobianExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

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

      DMatrixRMaj tmpMatrix = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.mult(weightMatrix, invertedFeedbackGain, tmpMatrix);
      CommonOps_DDRM.mult(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.quadraticTerm);

      tmpMatrix = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(weightMatrix, currentICPError, tmpMatrix);
      CommonOps_DDRM.multTransA(invertedFeedbackGain, tmpMatrix, icpQPInputExpected.linearTerm);
      CommonOps_DDRM.multTransA(0.5, currentICPError, tmpMatrix, icpQPInputExpected.residualCost);

      feedbackJacobianExpected.set(invertedFeedbackGain);
      feedbackObjectiveExpected.set(currentICPError);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 2);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 2, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps_DDRM.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps_DDRM.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 4);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 4);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      tmpMatrix = new DMatrixRMaj(4, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);

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

      feedbackJacobianExpected = new DMatrixRMaj(2, 6);
      feedbackObjectiveExpected = new DMatrixRMaj(2, 1);
      adjustmentJacobianExpected = new DMatrixRMaj(2, 6);
      adjustmentObjectiveExpected = new DMatrixRMaj(2, 1);

      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(feedbackJacobianExpected, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      feedbackObjectiveExpected.set(currentICPError);

      MatrixTools.setMatrixBlock(adjustmentJacobianExpected, 0, 4, CommonOps_DDRM.identity(2), 0, 0, 2, 2, footstepRecursionMultiplier);
      CommonOps_DDRM.add(footstepRecursionMultiplier, referenceFootstepLocation, currentICPError, adjustmentObjectiveExpected);

      tmpMatrix = new DMatrixRMaj(6, 2);
      CommonOps_DDRM.multTransA(feedbackJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(tmpMatrix, feedbackJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.mult(tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.linearTerm);

      CommonOps_DDRM.multTransA(adjustmentJacobianExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentJacobianExpected, icpQPInputExpected.quadraticTerm);
      CommonOps_DDRM.multAdd(tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.linearTerm);

      tmpMatrix = new DMatrixRMaj(1, 2);
      CommonOps_DDRM.multTransA(feedbackObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.mult(0.5, tmpMatrix, feedbackObjectiveExpected, icpQPInputExpected.residualCost);
      CommonOps_DDRM.multTransA(adjustmentObjectiveExpected, weightMatrix, tmpMatrix);
      CommonOps_DDRM.multAdd(0.5, tmpMatrix, adjustmentObjectiveExpected, icpQPInputExpected.residualCost);

      MatrixTestTools.assertMatrixEquals(feedbackJacobianExpected, inputCalculator.feedbackJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(feedbackObjectiveExpected, inputCalculator.feedbackObjective, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentJacobianExpected, inputCalculator.adjustmentJacobian, epsilon);
      MatrixTestTools.assertMatrixEquals(adjustmentObjectiveExpected, inputCalculator.adjustmentObjective, epsilon);

      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, epsilon);
      MatrixTestTools.assertMatrixEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, epsilon);
   }

   @Test
   public void testSubmitCoPeedbackTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);
      ICPQPInput feedbackRateTask = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      DMatrixRMaj feedbackRateWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackRateWeight);
      CommonOps_DDRM.scale(1.3, feedbackRateWeight);

      DMatrixRMaj previousFeedbackSolution = new DMatrixRMaj(2, 1);
      previousFeedbackSolution.set(0, 0, 0.03);
      previousFeedbackSolution.set(1, 0, 0.05);

      inputCalculator.computeCoPFeedbackRateTask(feedbackRateTask, feedbackRateWeight, previousFeedbackSolution);

      DMatrixRMaj quadratic = new DMatrixRMaj(2, 2);
      DMatrixRMaj linear = new DMatrixRMaj(2, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(2, 2);
      DMatrixRMaj linearExpected = new DMatrixRMaj(2, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackRateTask, quadratic, linear, scalar);

      quadraticExpected.set(feedbackTask.quadraticTerm);
      linearExpected.set(feedbackTask.linearTerm);
      scalarExpected.set(feedbackTask.residualCost);

      CommonOps_DDRM.addEquals(quadraticExpected, feedbackRateTask.quadraticTerm);
      CommonOps_DDRM.addEquals(linearExpected, feedbackRateTask.linearTerm);
      CommonOps_DDRM.addEquals(scalarExpected, feedbackRateTask.residualCost);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);


   }

   @Test
   public void testSubmitFeedbackRateTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackRateTask = new ICPQPInput(4);

      DMatrixRMaj feedbackRateWeight = new DMatrixRMaj(2, 2);
      DMatrixRMaj feedbackCMPCoPRateWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackRateWeight);
      CommonOps_DDRM.scale(1.3, feedbackRateWeight);
      MatrixTools.setDiagonal(feedbackCMPCoPRateWeight, 4.7);

      DMatrixRMaj previousCoPFeedbackSolution = new DMatrixRMaj(2, 1);
      DMatrixRMaj previousCMPFeedbackSolution = new DMatrixRMaj(2, 1);
      DMatrixRMaj previousFeedbackSolution = new DMatrixRMaj(2, 1);
      previousCoPFeedbackSolution.set(0, 0, 0.06);
      previousCoPFeedbackSolution.set(1, 0, 0.13);
      previousCMPFeedbackSolution.set(0, 0, 0.03);
      previousCMPFeedbackSolution.set(1, 0, 0.07);

      CommonOps_DDRM.add(previousCoPFeedbackSolution, previousCMPFeedbackSolution, previousFeedbackSolution);

      inputCalculator.computeCoPFeedbackRateTask(feedbackRateTask, feedbackCMPCoPRateWeight, previousCoPFeedbackSolution);
      inputCalculator.computeCMPFeedbackRateTask(feedbackRateTask, feedbackCMPCoPRateWeight, previousCMPFeedbackSolution);
      inputCalculator.computeFeedbackRateTask(feedbackRateTask, feedbackRateWeight, previousFeedbackSolution);

      DMatrixRMaj quadratic = new DMatrixRMaj(4, 4);
      DMatrixRMaj linear = new DMatrixRMaj(4, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(4, 4);
      DMatrixRMaj linearExpected = new DMatrixRMaj(4, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitFeedbackRateTask(feedbackRateTask, quadratic, linear, scalar);

      quadraticExpected.set(feedbackRateTask.quadraticTerm);
      linearExpected.set(feedbackRateTask.linearTerm);
      scalarExpected.set(feedbackRateTask.residualCost);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, epsilon);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, epsilon);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, epsilon);

      quadraticExpected = new DMatrixRMaj(4, 4);

      quadraticExpected.set(0, 0, feedbackCMPCoPRateWeight.get(0, 0) + feedbackRateWeight.get(0, 0));
      quadraticExpected.set(1, 1, feedbackCMPCoPRateWeight.get(1, 1) + feedbackRateWeight.get(1, 1));
      quadraticExpected.set(2, 2, feedbackCMPCoPRateWeight.get(0, 0) + feedbackRateWeight.get(0, 0));
      quadraticExpected.set(3, 3, feedbackCMPCoPRateWeight.get(1, 1) + feedbackRateWeight.get(1, 1));
      quadraticExpected.set(0, 2, feedbackRateWeight.get(0, 0));
      quadraticExpected.set(1, 3, feedbackRateWeight.get(1, 1));
      quadraticExpected.set(2, 0, feedbackRateWeight.get(0, 0));
      quadraticExpected.set(3, 1, feedbackRateWeight.get(1, 1));

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, epsilon);

   }

   @Test
   public void testSubmitDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj dynamicsWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(dynamicsWeight);
      CommonOps_DDRM.scale(2.7, dynamicsWeight);

      DMatrixRMaj feedbackGains = new DMatrixRMaj(2, 2);
      DMatrixRMaj invertedFeedbackGains = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGains);
      CommonOps_DDRM.scale(2.7, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      DMatrixRMaj quadratic = new DMatrixRMaj(6, 6);
      DMatrixRMaj linear = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj linearExpected = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);

      quadraticExpected.set(dynamicsTask.quadraticTerm);
      linearExpected.set(dynamicsTask.linearTerm);
      scalarExpected.set(dynamicsTask.residualCost);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitCMPFeedbackTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DMatrixRMaj angularMomentumWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(angularMomentumWeight);
      CommonOps_DDRM.scale(2.0, angularMomentumWeight);

      inputCalculator.computeCMPFeedbackTask(angularMomentumTask, angularMomentumWeight);

      DMatrixRMaj quadratic = new DMatrixRMaj(4, 4);
      DMatrixRMaj linear = new DMatrixRMaj(4, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(4, 4);
      DMatrixRMaj linearExpected = new DMatrixRMaj(4, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitCMPFeedbackTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      CommonOps_DDRM.addEquals(scalarExpected, angularMomentumTask.residualCost);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      ICPQPInput footstepTask = new ICPQPInput(8);
      ICPQPInput footstepRateTask = new ICPQPInput(8);

      DMatrixRMaj footstepObjective1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective2 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective3 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepObjective4 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious2 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious3 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious4 = new DMatrixRMaj(2, 1);

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

      DMatrixRMaj footstepWeight = new DMatrixRMaj(2, 2);
      DMatrixRMaj footstepRateWeight = new DMatrixRMaj(2, 2);

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

      DMatrixRMaj quadratic = new DMatrixRMaj(10, 10);
      DMatrixRMaj linear = new DMatrixRMaj(10, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(10, 10);
      DMatrixRMaj linearExpected = new DMatrixRMaj(10, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRateTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 2, 2, footstepTask.quadraticTerm, 0, 0, 8, 8, 1.0);
      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, footstepRateTask.quadraticTerm, 0, 0, 8, 8, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 2, 0, footstepTask.linearTerm, 0, 0, 8, 1, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, footstepRateTask.linearTerm, 0, 0, 8, 1, 1.0);
      scalarExpected.set(footstepTask.residualCost);
      CommonOps_DDRM.addEquals(scalarExpected, footstepRateTask.residualCost);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitCoPAndCMPOFeedbackTasks()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DMatrixRMaj angularMomentumWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(angularMomentumWeight);
      CommonOps_DDRM.scale(2.0, angularMomentumWeight);

      inputCalculator.computeCMPFeedbackTask(angularMomentumTask, angularMomentumWeight);

      DMatrixRMaj quadratic = new DMatrixRMaj(4, 4);
      DMatrixRMaj linear = new DMatrixRMaj(4, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(4, 4);
      DMatrixRMaj linearExpected = new DMatrixRMaj(4, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitCMPFeedbackTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, angularMomentumTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj dynamicsWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(dynamicsWeight);
      CommonOps_DDRM.scale(2.7, dynamicsWeight);

      DMatrixRMaj feedbackGains = new DMatrixRMaj(2, 2);
      DMatrixRMaj invertedFeedbackGains = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGains);
      CommonOps_DDRM.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      DMatrixRMaj quadratic = new DMatrixRMaj(6, 6);
      DMatrixRMaj linear = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj linearExpected = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitCoPFeedbackTask(feedbackTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.registerFootstep();
      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRateTask = new ICPQPInput(2);

      DMatrixRMaj footstepObjective1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious1 = new DMatrixRMaj(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DMatrixRMaj footstepWeight = new DMatrixRMaj(2, 2);
      DMatrixRMaj footstepRateWeight = new DMatrixRMaj(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);

      DMatrixRMaj quadratic = new DMatrixRMaj(6, 6);
      DMatrixRMaj linear = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj linearExpected = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

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

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndFootstepAndDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj dynamicsWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(dynamicsWeight);
      CommonOps_DDRM.scale(2.7, dynamicsWeight);

      DMatrixRMaj feedbackGains = new DMatrixRMaj(2, 2);
      DMatrixRMaj invertedFeedbackGains = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGains);
      CommonOps_DDRM.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRateTask = new ICPQPInput(2);

      DMatrixRMaj footstepObjective1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious1 = new DMatrixRMaj(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DMatrixRMaj footstepWeight = new DMatrixRMaj(2, 2);
      DMatrixRMaj footstepRateWeight = new DMatrixRMaj(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);
      DMatrixRMaj quadratic = new DMatrixRMaj(6, 6);
      DMatrixRMaj linear = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj linearExpected = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

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

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, 1e-7);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, 1e-7);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, 1e-7);
   }

   @Test
   public void testSubmitFeedbackAndFootstepAndDynamicsAndAngularMomentumTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setHasCMPFeedbackTask(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DMatrixRMaj feedbackWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackWeight);
      CommonOps_DDRM.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeCoPFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DMatrixRMaj currentICPError = new DMatrixRMaj(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DMatrixRMaj referenceFootstepLocation = new DMatrixRMaj(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DMatrixRMaj dynamicsWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(dynamicsWeight);
      CommonOps_DDRM.scale(2.7, dynamicsWeight);

      DMatrixRMaj feedbackGains = new DMatrixRMaj(2, 2);
      DMatrixRMaj invertedFeedbackGains = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(feedbackGains);
      CommonOps_DDRM.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier,
                                          safetyFactor);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRateTask = new ICPQPInput(2);

      DMatrixRMaj footstepObjective1 = new DMatrixRMaj(2, 1);
      DMatrixRMaj footstepPrevious1 = new DMatrixRMaj(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DMatrixRMaj footstepWeight = new DMatrixRMaj(2, 2);
      DMatrixRMaj footstepRateWeight = new DMatrixRMaj(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRateWeight.set(0, 0, 0.1);
      footstepRateWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRateTask(0, footstepRateTask, footstepRateWeight, footstepPrevious1);

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DMatrixRMaj angularMomentumWeight = new DMatrixRMaj(2, 2);
      CommonOps_DDRM.setIdentity(angularMomentumWeight);
      CommonOps_DDRM.scale(2.0, angularMomentumWeight);

      inputCalculator.computeCMPFeedbackTask(angularMomentumTask, angularMomentumWeight);

      DMatrixRMaj quadratic = new DMatrixRMaj(6, 6);
      DMatrixRMaj linear = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj quadraticExpected = new DMatrixRMaj(6, 6);
      DMatrixRMaj linearExpected = new DMatrixRMaj(6, 1);
      DMatrixRMaj scalarExpected = new DMatrixRMaj(1, 1);

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

      MatrixTestTools.assertMatrixEquals(quadraticExpected, quadratic, epsilon);
      MatrixTestTools.assertMatrixEquals(linearExpected, linear, epsilon);
      MatrixTestTools.assertMatrixEquals(scalarExpected, scalar, epsilon);
   }

   @Test
   public void testComputeDynamicsConstraintError()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);
      Random random = new Random(1738L);

      for (int iter = 0; iter < 100; iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 100);
         DMatrixRMaj solution = new DMatrixRMaj(size, 1);

         DMatrixRMaj feedbackJacobian = RandomMatrices_DDRM.rectangle(2, size, random);
         DMatrixRMaj adjustmentJacobian = RandomMatrices_DDRM.rectangle(2, size, random);

         DMatrixRMaj feedbackObjective = RandomMatrices_DDRM.rectangle(2, 1, random);
         DMatrixRMaj adjustmentObjective = RandomMatrices_DDRM.rectangle(2, 1, random);

         inputCalculator.feedbackJacobian.set(feedbackJacobian);
         inputCalculator.adjustmentJacobian.set(adjustmentJacobian);

         inputCalculator.feedbackObjective.set(feedbackObjective);
         inputCalculator.adjustmentObjective.set(adjustmentObjective);

         DMatrixRMaj errorToTest = new DMatrixRMaj(2, 1);
         DMatrixRMaj errorExpected = new DMatrixRMaj(2, 1);

         CommonOps_DDRM.multAdd(feedbackJacobian, solution, errorExpected);
         CommonOps_DDRM.multAdd(adjustmentJacobian, solution, errorExpected);
         CommonOps_DDRM.addEquals(errorExpected, -1.0, feedbackObjective);
         CommonOps_DDRM.addEquals(errorExpected, -1.0, adjustmentObjective);

         inputCalculator.computeDynamicConstraintError(solution, errorToTest);

         MatrixTestTools.assertMatrixEquals(errorExpected, errorToTest, epsilon);
      }

   }

   private static void assertInputEquals(ICPQPInput inputA, ICPQPInput inputB, double tol)
   {
      MatrixTestTools.assertMatrixEquals("Quadratic terms aren't equal.", inputA.quadraticTerm, inputB.quadraticTerm, tol);
      MatrixTestTools.assertMatrixEquals("Linear terms aren't equal.", inputA.linearTerm, inputB.linearTerm, tol);
      MatrixTestTools.assertMatrixEquals("Residual terms aren't equal.", inputA.residualCost, inputB.residualCost, tol);
   }


   private static DMatrixRMaj computeCost(ICPQPInput icpqpInput, DMatrixRMaj solution)
   {
      DMatrixRMaj tempMatrix = new DMatrixRMaj(solution.numRows, 1);
      DMatrixRMaj cost = new DMatrixRMaj(1, 1);

      CommonOps_DDRM.mult(icpqpInput.quadraticTerm, solution, tempMatrix);
      CommonOps_DDRM.multTransA(solution, tempMatrix, cost);
      CommonOps_DDRM.scale(0.5, cost);

      CommonOps_DDRM.multAddTransA(-1.0, icpqpInput.linearTerm, solution, cost);
      CommonOps_DDRM.addEquals(cost, icpqpInput.residualCost);

      return cost;
   }
}
