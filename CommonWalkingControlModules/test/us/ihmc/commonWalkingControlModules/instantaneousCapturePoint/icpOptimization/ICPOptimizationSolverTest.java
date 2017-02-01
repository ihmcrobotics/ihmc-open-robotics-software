package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import static org.junit.Assert.assertEquals;

import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Random;

public class ICPOptimizationSolverTest extends ICPOptimizationSolver
{
   private static final YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRobert");

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.001;
   private static final double omega = 3.5;

   public ICPOptimizationSolverTest()
   {
      super(icpOptimizationParameters, 20, rootRegistry);
      rootRegistry.clear();
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testSetFeedbackConditions()
   {
      Random random = new Random();
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         double feedbackWeight = 10.0 * random.nextDouble();
         double feedbackGain = 10.0 * random.nextDouble();

         super.setFeedbackConditions(feedbackWeight, feedbackGain, 1000.0);

         checkFeedbackMatrices(feedbackWeight, feedbackGain);
      }
   }

   private void checkFeedbackMatrices(double feedbackWeight, double feedbackGain)
   {
      assertEquals("", feedbackWeight, this.feedbackWeight.get(0, 0), epsilon);
      assertEquals("", feedbackWeight, this.feedbackWeight.get(1, 1), epsilon);

      assertEquals("", feedbackGain, this.feedbackGain.get(1, 1), epsilon);
      assertEquals("", feedbackGain, this.feedbackGain.get(1, 1), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testDimensions()
   {
      for (int i = 1; i < this.maximumNumberOfFootstepsToConsider; i++)
      {
         testDimension(i, 0, false, true, false);
         testDimension(i, 0, true, true, false);
         testDimension(i, 0, true, false, false);

         testDimension(i, 4, false, true, false);
         testDimension(i, 4, true, true, false);
         testDimension(i, 4, true, false, false);

         testDimension(i, 8, false, true, false);
         testDimension(i, 8, true, true, false);
         testDimension(i, 8, true, false, false);
      }
   }

   private void testDimension(int numberOfFootstepsToConsider, int numberOfVertices, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      super.setNumberOfCMPVertices(numberOfVertices);
      super.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs);

      int numberOfLagrangeMultipliers = 2;
      int numberOfFootstepVariables = 0;

      int feedbackTaskIndex = 0;
      int dynamicRelaxationIndex = 0;

      if (useStepAdjustment)
      {
         numberOfFootstepVariables = 2 * numberOfFootstepsToConsider;
         dynamicRelaxationIndex += numberOfFootstepVariables;
      }

      int totalNumberOfFreeVariables = numberOfFootstepVariables + 2;

      if (useFeedback)
      {
         totalNumberOfFreeVariables += 2;

         if (numberOfVertices > 0)
            numberOfLagrangeMultipliers += 3;
      }
      else
      {
         numberOfVertices = 0;
      }

      String name = "Number of Steps: " + numberOfFootstepsToConsider + ". Use step adjustment: " + useStepAdjustment + ". Use Feedback: " + useFeedback;

      assertEquals(name, numberOfFootstepVariables, this.numberOfFootstepVariables, epsilon);
      assertEquals(name, numberOfLagrangeMultipliers, this.numberOfLagrangeMultipliers, epsilon);
      assertEquals(name, totalNumberOfFreeVariables, this.numberOfFreeVariables, epsilon);
      assertEquals(name, numberOfVertices, this.numberOfCMPVertices, epsilon);

      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_H.numRows, epsilon);
      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_H.numCols, epsilon);
      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_h.numRows, epsilon);
      assertEquals(name, 1, solverInput_h.numCols, epsilon);

      /*
      assertEquals(name, numberOfFootstepVariables, footstepCost_H.numRows, epsilon);
      assertEquals(name, numberOfFootstepVariables, footstepCost_H.numCols, epsilon);
      assertEquals(name, numberOfFootstepVariables, footstepCost_h.numRows, epsilon);
      assertEquals(name, 1, footstepCost_h.numCols, epsilon);
      */

      /*
      assertEquals(name, numberOfFootstepVariables, footstepRegularizationCost_H.numRows, epsilon);
      assertEquals(name, numberOfFootstepVariables, footstepRegularizationCost_H.numCols, epsilon);
      assertEquals(name, numberOfFootstepVariables, footstepRegularizationCost_h.numRows, epsilon);
      assertEquals(name, 1, footstepRegularizationCost_h.numCols, epsilon);
      */

      /*
      if (useFeedback)
      {
         assertEquals(name, this.numberOfCMPVertices, stanceCMPCost_G.numRows, epsilon);
         assertEquals(name, this.numberOfCMPVertices, stanceCMPCost_G.numCols, epsilon);
      }
      */

      /*
      assertEquals(name, 2, feedbackCost_H.numRows, epsilon);
      assertEquals(name, 2, feedbackCost_H.numCols, epsilon);
      assertEquals(name, 2, feedbackCost_h.numRows, epsilon);
      assertEquals(name, 1, feedbackCost_h.numCols, epsilon);

      assertEquals(name, 2, feedbackRegularizationCost_H.numRows, epsilon);
      assertEquals(name, 2, feedbackRegularizationCost_H.numCols, epsilon);
      assertEquals(name, 2, feedbackRegularizationCost_h.numRows, epsilon);
      assertEquals(name, 1, feedbackRegularizationCost_h.numCols, epsilon);

      assertEquals(name, 2, dynamicRelaxationCost_H.numRows, epsilon);
      assertEquals(name, 2, dynamicRelaxationCost_H.numCols, epsilon);
      assertEquals(name, 2, dynamicRelaxationCost_h.numRows, epsilon);
      assertEquals(name, 1, dynamicRelaxationCost_h.numCols, epsilon);
      */

      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_Aeq.numRows, epsilon);
      assertEquals(name, numberOfLagrangeMultipliers, solverInput_Aeq.numCols, epsilon);
      assertEquals(name, numberOfLagrangeMultipliers, solverInput_AeqTrans.numRows, epsilon);
      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_AeqTrans.numCols, epsilon);
      assertEquals(name, numberOfLagrangeMultipliers, solverInput_beq.numRows, epsilon);
      assertEquals(name, 1, solverInput_beq.numCols, epsilon);

      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_Aineq.numRows, epsilon);
      assertEquals(name, numberOfVertices, solverInput_Aineq.numCols, epsilon);
      assertEquals(name, numberOfVertices, solverInput_AineqTrans.numRows, epsilon);
      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, solverInput_AineqTrans.numCols, epsilon);
      assertEquals(name, numberOfVertices, solverInput_bineq.numRows, epsilon);
      assertEquals(name, 1, solverInput_bineq.numCols, epsilon);

      /*
      assertEquals(name, totalNumberOfFreeVariables + numberOfVertices, dynamics_Aeq.numRows, epsilon);
      assertEquals(name, 2, dynamics_Aeq.numCols, epsilon);
      assertEquals(name, 2, dynamics_beq.numRows, epsilon);
      assertEquals(name, 1, dynamics_beq.numCols, epsilon);
      */

      /*
      if (useFeedback)
      {
         assertEquals(name, 4 + numberOfVertices, stanceCMP_Aeq.numRows, epsilon);
         assertEquals(name, 3, stanceCMP_Aeq.numCols, epsilon);
         assertEquals(name, 3, stanceCMP_beq.numRows, epsilon);
         assertEquals(name, 1, stanceCMP_beq.numCols, epsilon);

         assertEquals(name, 4 + numberOfVertices, stanceCMPDynamics_Aeq.numRows, epsilon);
         assertEquals(name, 2, stanceCMPDynamics_Aeq.numCols, epsilon);
         assertEquals(name, 2, stanceCMPDynamics_beq.numRows, epsilon);
         assertEquals(name, 1, stanceCMPDynamics_beq.numCols, epsilon);

         assertEquals(name, numberOfVertices, stanceCMPSum_Aeq.numRows, epsilon);
         assertEquals(name, 1, stanceCMPSum_Aeq.numCols, epsilon);
         assertEquals(name, 1, stanceCMPSum_beq.numRows, epsilon);
         assertEquals(name, 1, stanceCMPSum_beq.numCols, epsilon);

         assertEquals(name, numberOfVertices, stanceCMP_Aineq.numRows, epsilon);
         assertEquals(name, numberOfVertices, stanceCMP_Aineq.numCols, epsilon);
         assertEquals(name, numberOfVertices, stanceCMP_bineq.numRows, epsilon);
         assertEquals(name, 1, stanceCMP_bineq.numCols, epsilon);
      }
      */
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testConditionError()
   {
      testCondition(0, true, false, true);
      testCondition(0, true, false, false);
      testCondition(0, false, false, true);
      testCondition(0, false, false, false);
      testCondition(1, false, false, true);
      testCondition(1, false, false, false);
   }

   public void testCondition(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      boolean hasError = false;
      try
      {
         super.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs);
      }
      catch (RuntimeException e)
      {
         hasError = true;
      }

      Assert.assertTrue(hasError);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDynamicConstraint()
   {
      double omega = 3.0;
      double remainingTime = 0.5;
      double feedbackGain = 2.0;

      double doubleSupportDuration = 0.2;
      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      int numberOffootstepsToConsider = 0;

      super.submitProblemConditions(numberOffootstepsToConsider, true, true, false);
      super.setFeedbackConditions(0.001, feedbackGain, 1000.0);

      double finalICPRecursionMultiplier = Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d finalICP = new FramePoint2d(worldFrame, 0.2, 0.115);
      FramePoint2d finalICPRecursion = new FramePoint2d();
      finalICPRecursion.set(finalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      double currentStateProjection = Math.exp(omega * remainingTime);
      double cmpProjectionMultiplier = currentStateProjection - Math.exp(-omega * initialDoubleSupportDuration);

      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.0, -0.155);
      FramePoint2d cmpProjection = new FramePoint2d();
      FramePoint2d initialICPProjection = new FramePoint2d();

      cmpProjection.set(perfectCMP);
      cmpProjection.scale(cmpProjectionMultiplier);


      FramePoint2d currentICP = new FramePoint2d(worldFrame, 0.1, 0.06);

      initialICPProjection.set(currentICP);
      initialICPProjection.scale(currentStateProjection);

      try
      {
         this.compute(finalICPRecursion, null, currentICP, perfectCMP, cmpProjection, initialICPProjection);
      }
      catch (NoConvergenceException e)
      {
      }

      FramePoint2d rightHandSide = new FramePoint2d();
      rightHandSide.set(currentICP);
      rightHandSide.sub(cmpProjection);
      rightHandSide.sub(finalICPRecursion);
      rightHandSide.sub(initialICPProjection);

      DenseMatrix64F expectedDynamics_beq = new DenseMatrix64F(2, 1);
      expectedDynamics_beq.set(0, 0, rightHandSide.getX());
      expectedDynamics_beq.set(1, 0, rightHandSide.getY());

      DenseMatrix64F expectedDynamics_Aeq = new DenseMatrix64F(4, 2);
      DenseMatrix64F identity = CommonOps.identity(2, 2);

      MatrixTools.setMatrixBlock(expectedDynamics_Aeq, 0, 0, identity, 0, 0, 2, 2, 1.0 / feedbackGain);
      MatrixTools.setMatrixBlock(expectedDynamics_Aeq, 2, 0, identity, 0, 0, 2, 2, 1.0);

      /*
      JUnitTools.assertMatrixEquals(expectedDynamics_beq, dynamics_beq, epsilon);
      JUnitTools.assertMatrixEquals(expectedDynamics_Aeq, dynamics_Aeq, epsilon);
      */

      DenseMatrix64F extracted_Aeq = new DenseMatrix64F(4, 2);
      DenseMatrix64F extracted_beq = new DenseMatrix64F(2, 1);

      CommonOps.extract(solverInput_Aeq, 0, 4, 0, 2, extracted_Aeq, 0, 0);
      CommonOps.extract(solverInput_beq, 0, 2, 0, 1, extracted_beq, 0, 0);

      JUnitTools.assertMatrixEquals(expectedDynamics_Aeq, extracted_Aeq, epsilon);
      JUnitTools.assertMatrixEquals(expectedDynamics_beq, extracted_beq, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testConstraintAssembly()
   {
      double omega = 3.0;
      double remainingTime = 0.5;
      double feedbackGain = 2.0;

      double doubleSupportDuration = 0.2;
      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      int numberOffootstepsToConsider = 2;
      int numberOfVertices = 4;

      super.setNumberOfCMPVertices(numberOfVertices);
      super.submitProblemConditions(numberOffootstepsToConsider, true, true, false);
      super.setFeedbackConditions(0.001, feedbackGain, 1000.0);

      for (int i = 0; i < numberOffootstepsToConsider; i++)
      {
         super.setFootstepWeight(i, 1.0, 1.0);
      }

      double finalICPRecursionMultiplier = Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d finalICP = new FramePoint2d(worldFrame, 0.2, 0.115);
      FramePoint2d finalICPRecursion = new FramePoint2d();
      finalICPRecursion.set(finalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      double currentStateProjection = Math.exp(omega * remainingTime);
      double cmpProjectionMultiplier = currentStateProjection - Math.exp(-omega * initialDoubleSupportDuration);

      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.0, -0.155);
      FramePoint2d cmpProjection = new FramePoint2d();
      FramePoint2d initialICPProjection = new FramePoint2d();

      cmpProjection.set(perfectCMP);
      cmpProjection.scale(cmpProjectionMultiplier);

      FramePoint2d currentICP = new FramePoint2d(worldFrame, 0.1, 0.06);

      initialICPProjection.set(currentICP);
      initialICPProjection.scale(currentStateProjection);

      try
      {
         this.compute(finalICPRecursion, null, currentICP, perfectCMP, cmpProjection, initialICPProjection);
      }
      catch (NoConvergenceException e)
      {
      }

      DenseMatrix64F extractedDynamics_Aeq = new DenseMatrix64F(4 + 2 * numberOffootstepsToConsider + numberOfVertices, 2);
      DenseMatrix64F extractedDynamics_beq = new DenseMatrix64F(2, 1);

      DenseMatrix64F extractedCMPDynamics_Aeq = new DenseMatrix64F(4 + numberOfVertices, 2);
      DenseMatrix64F extractedCMPDynamics_beq = new DenseMatrix64F(2, 1);

      DenseMatrix64F extractedCMPSum_Aeq = new DenseMatrix64F(numberOfVertices, 1);
      DenseMatrix64F extractedCMPSum_beq = new DenseMatrix64F(1, 1);

      CommonOps.extract(solverInput_Aeq, 0, 4 + 2 * numberOffootstepsToConsider + numberOfVertices, 3, 5, extractedDynamics_Aeq, 0, 0);
      CommonOps.extract(solverInput_beq, 3, 5, 0, 1, extractedDynamics_beq, 0, 0);

      CommonOps.extract(solverInput_Aeq, 2 * numberOffootstepsToConsider, 2*numberOffootstepsToConsider + numberOfVertices, 0, 2, extractedCMPDynamics_Aeq, 0, 0);
      CommonOps.extract(solverInput_beq, 0, 2, 0, 1, extractedCMPDynamics_beq, 0, 0);

      CommonOps.extract(solverInput_Aeq, 2 * numberOffootstepsToConsider + 4, 2 * numberOffootstepsToConsider + numberOfVertices + 4, 2, 3, extractedCMPSum_Aeq, 0, 0);
      CommonOps.extract(solverInput_beq, 2, 3, 0, 1, extractedCMPSum_beq, 0, 0);

      /*
      JUnitTools.assertMatrixEquals(dynamics_Aeq, extractedDynamics_Aeq, epsilon);
      JUnitTools.assertMatrixEquals(dynamics_beq, extractedDynamics_beq, epsilon);
      */

      /*
      JUnitTools.assertMatrixEquals(stanceCMPDynamics_Aeq, extractedCMPDynamics_Aeq, epsilon);
      JUnitTools.assertMatrixEquals(stanceCMPDynamics_beq, extractedCMPDynamics_beq, epsilon);

      JUnitTools.assertMatrixEquals(stanceCMPSum_Aeq, extractedCMPSum_Aeq, epsilon);
      JUnitTools.assertMatrixEquals(stanceCMPSum_beq, extractedCMPSum_beq, epsilon);
      */
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testAddFeedbackTask()
   {
      double feedbackWeight = 2.0;
      double feedbackGain = 0.001;

      int numberOfVertices = 4;

      for (int numberOfFootstepsToConsider = 0; numberOfFootstepsToConsider < maximumNumberOfFootstepsToConsider; numberOfFootstepsToConsider++)
      {
         int feedbackIndex = 2 * numberOfFootstepsToConsider;

         super.submitProblemConditions(numberOfFootstepsToConsider, true, true, false); //, false);
         super.setFeedbackConditions(feedbackWeight, feedbackGain, 1000.0);

         checkFeedbackMatrices(feedbackWeight, feedbackGain);
         testDimension(numberOfFootstepsToConsider, numberOfVertices, true, true, false);

         super.addFeedbackTask();

         DenseMatrix64F weightBlock = new DenseMatrix64F(2, 2);
         CommonOps.extract(solverInput_H, feedbackIndex, feedbackIndex + 2, feedbackIndex, feedbackIndex + 2, weightBlock, 0, 0);

         String name = "Step number " + numberOfFootstepsToConsider;
         //JUnitTools.assertMatrixEquals(name, feedbackCost_H, weightBlock, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSetFootstepAdjustmentConditions()
   {
      Random random = new Random();
      double recursionMultiplier = 0.05 * random.nextDouble();
      double weight = 10.0 * random.nextDouble();
      double stepLength = 0.5 * random.nextDouble();
      double stepWidth = 0.1 * random.nextDouble();

      int numberOfVertices = 4;

      for (int numberOfFootstepsToConsider = 1; numberOfFootstepsToConsider < maximumNumberOfFootstepsToConsider; numberOfFootstepsToConsider++)
      {
         super.submitProblemConditions(numberOfFootstepsToConsider, true, false, false); //, false);
         testDimension(numberOfFootstepsToConsider, numberOfVertices, true, true, false);

         RobotSide stepSide = RobotSide.LEFT;

         ArrayList<FramePoint2d> localReferenceFootstepLocations = new ArrayList<>();
         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
         {
            double xPosition = (footstepIndex + 1) * stepLength;
            double yPosition = stepSide.negateIfRightSide(stepWidth);
            FramePoint2d referenceFootstepLocation = new FramePoint2d();
            referenceFootstepLocation.set(xPosition, yPosition);
            localReferenceFootstepLocations.add(referenceFootstepLocation);

            super.setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, weight, referenceFootstepLocation);


            stepSide = stepSide.getOppositeSide();
         }

         super.addStepAdjustmentTask();

         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            checkRecursionMultiplier(i, recursionMultiplier);
            checkReferenceFootstepLocation(i, localReferenceFootstepLocations.get(i));
            checkFootstepWeight(i, weight, localReferenceFootstepLocations.get(i));
         }


         DenseMatrix64F footstepWeightBlock = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
         DenseMatrix64F footstepEqualsBlock = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

         CommonOps.extract(solverInput_H, 0, 2 * numberOfFootstepsToConsider, 0, 2 * numberOfFootstepsToConsider, footstepWeightBlock, 0, 0);
         CommonOps.extract(solverInput_h, 0, 2 * numberOfFootstepsToConsider, 0, 1, footstepEqualsBlock, 0, 0);


         /*
         JUnitTools.assertMatrixEquals(footstepCost_H, footstepWeightBlock, epsilon);
         JUnitTools.assertMatrixEquals(footstepCost_h, footstepEqualsBlock, epsilon);
         */
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSetFootstepRegularizationConditions()
   {
      Random random = new Random();
      double recursionMultiplier = 0.05 * random.nextDouble();
      double weight = 10.0 * random.nextDouble();
      double stepLength = 0.5 * random.nextDouble();
      double stepWidth = 0.1 * random.nextDouble();

      int numberOfVertices = 4;

      for (int numberOfFootstepsToConsider = 1; numberOfFootstepsToConsider < maximumNumberOfFootstepsToConsider; numberOfFootstepsToConsider++)
      {
         super.submitProblemConditions(numberOfFootstepsToConsider, true, false, false);
         testDimension(numberOfFootstepsToConsider, numberOfVertices, true, true, false);

         RobotSide stepSide = RobotSide.LEFT;

         ArrayList<DenseMatrix64F> localPreviousFootstepSolution = new ArrayList<>();
         ArrayList<FramePoint2d> localReferenceFootstepLocations = new ArrayList<>();
         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
         {
            double xPosition = (footstepIndex + 1) * stepLength;
            double yPosition = stepSide.negateIfRightSide(stepWidth);
            FramePoint2d referenceFootstepLocation = new FramePoint2d();
            referenceFootstepLocation.set(xPosition, yPosition);
            localReferenceFootstepLocations.add(referenceFootstepLocation);

            DenseMatrix64F previousFootstepLocation = new DenseMatrix64F(2, 1);
            previousFootstepLocation.set(0, 0, xPosition);
            previousFootstepLocation.set(1, 0, yPosition);

            super.setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, 0.0, referenceFootstepLocation);
            super.setFootstepRegularizationWeight(weight);
            super.resetFootstepRegularization(footstepIndex, referenceFootstepLocation);

            localPreviousFootstepSolution.add(new DenseMatrix64F(2, 1));
            localPreviousFootstepSolution.get(footstepIndex).set(0, 0, xPosition);
            localPreviousFootstepSolution.get(footstepIndex).set(1, 0, yPosition);

            JUnitTools.assertMatrixEquals(previousFootstepLocation, this.previousFootstepLocations.get(footstepIndex), epsilon);

            solution.set(footstepIndex * 2, xPosition);
            solution.set(footstepIndex * 2 + 1, yPosition);

            stepSide = stepSide.getOppositeSide();
         }

         super.addStepAdjustmentTask();
         //super.addFootstepRegularizationTask();

         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            checkRecursionMultiplier(i, recursionMultiplier);
            checkReferenceFootstepLocation(i, localReferenceFootstepLocations.get(i));
            checkFootstepRegularizationWeight(i, weight, localReferenceFootstepLocations.get(i));
         }

         super.setPreviousFootstepSolution(solution);

         DenseMatrix64F footstepWeightBlock = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
         DenseMatrix64F footstepEqualsBlock = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);

         CommonOps.extract(solverInput_H, 0, 2 * numberOfFootstepsToConsider, 0, 2 * numberOfFootstepsToConsider, footstepWeightBlock, 0, 0);
         CommonOps.extract(solverInput_h, 0, 2 * numberOfFootstepsToConsider, 0, 1, footstepEqualsBlock, 0, 0);

         /*
         JUnitTools.assertMatrixEquals(footstepRegularizationCost_H, footstepWeightBlock, epsilon);
         JUnitTools.assertMatrixEquals(footstepRegularizationCost_h, footstepEqualsBlock, epsilon);
         */

         for (int i = 0; i < numberOfFootstepsToConsider; i++)
            JUnitTools.assertMatrixEquals(localPreviousFootstepSolution.get(i), this.previousFootstepLocations.get(i) , epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testRegularization()
   {
      Random random = new Random();
      double weight = 10.0 * random.nextDouble();
      double stepLength = 0.5 * random.nextDouble();
      double stepWidth = 0.1 * random.nextDouble();
      double modifier = 0.1 * random.nextDouble();

      for (int numberOfFootstepsToConsider = 1; numberOfFootstepsToConsider < maximumNumberOfFootstepsToConsider; numberOfFootstepsToConsider++)
      {
         super.submitProblemConditions(numberOfFootstepsToConsider, true, false, false);

         RobotSide stepSide = RobotSide.LEFT;

         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
         {
            double xPosition = (footstepIndex + 1) * stepLength;
            double yPosition = stepSide.negateIfRightSide(stepWidth);
            FramePoint2d referenceFootstepLocation = new FramePoint2d();
            referenceFootstepLocation.set(xPosition, yPosition);

            super.setFootstepRegularizationWeight(weight);
            super.resetFootstepRegularization(footstepIndex, referenceFootstepLocation);

            solution.set(footstepIndex * 2, xPosition + modifier);
            solution.set(footstepIndex * 2 + 1, yPosition + 1);

            stepSide = stepSide.getOppositeSide();
         }

         super.setPreviousFootstepSolution(solution);
         //super.addFootstepRegularizationTask();

         DenseMatrix64F shouldBe_H = CommonOps.identity(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
         CommonOps.scale(weight, shouldBe_H);

         DenseMatrix64F shouldBe_h = new DenseMatrix64F(2 * numberOfFootstepsToConsider, 1);
         DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 1);
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            tmpObjective.set(0, 0, solution.get(2 * i, 0));
            tmpObjective.set(1, 0, solution.get(2 * i + 1, 0));

            MatrixTools.setMatrixBlock(shouldBe_h, 2 * i, 0, tmpObjective, 0, 0, 2, 1, weight);
         }

         /*
         JUnitTools.assertMatrixEquals(shouldBe_H, footstepRegularizationCost_H, epsilon);
         JUnitTools.assertMatrixEquals(shouldBe_h, footstepRegularizationCost_h, epsilon);
         */
      }
   }

   private void checkFootstepWeight(int footstepIndex, double footstepWeight, FramePoint2d referenceFootstepLocation)
   {
      DenseMatrix64F location = new DenseMatrix64F(2, 1);
      location.set(0, 0, referenceFootstepLocation.getX());
      location.set(1, 0, referenceFootstepLocation.getY());

      DenseMatrix64F weight = CommonOps.identity(2, 2);
      CommonOps.scale(footstepWeight, weight);

      DenseMatrix64F equals = new DenseMatrix64F(2, 1);
      equals.set(location);
      CommonOps.mult(weight, equals, equals);

      JUnitTools.assertMatrixEquals("", weight, footstepWeights.get(footstepIndex), epsilon);

      int index = 2 * footstepIndex;
      DenseMatrix64F weightBlock = new DenseMatrix64F(2, 2);
      DenseMatrix64F equalsBlock = new DenseMatrix64F(2, 1);

      /*
      CommonOps.extract(footstepCost_H, index, index + 2, index, index + 2, weightBlock, 0, 0);
      CommonOps.extract(footstepCost_h, index, index + 2, 0, 1, equalsBlock, 0, 0);
      */

      JUnitTools.assertMatrixEquals("", weight, weightBlock, epsilon);
      JUnitTools.assertMatrixEquals("", equals, equalsBlock, epsilon);
   }

   private void checkFootstepRegularizationWeight(int footstepIndex, double footstepWeight, FramePoint2d referenceFootstepLocation)
   {
      DenseMatrix64F location = new DenseMatrix64F(2, 1);
      location.set(0, 0, referenceFootstepLocation.getX());
      location.set(1, 0, referenceFootstepLocation.getY());

      DenseMatrix64F weight = CommonOps.identity(2, 2);
      CommonOps.scale(footstepWeight, weight);

      DenseMatrix64F equals = new DenseMatrix64F(2, 1);
      equals.set(location);
      CommonOps.mult(weight, equals, equals);

      int index = 2 * footstepIndex;
      DenseMatrix64F weightBlock = new DenseMatrix64F(2, 2);
      DenseMatrix64F equalsBlock = new DenseMatrix64F(2, 1);

      /*
      CommonOps.extract(footstepRegularizationCost_H, index, index + 2, index, index + 2, weightBlock, 0, 0);
      CommonOps.extract(footstepRegularizationCost_h, index, index + 2, 0, 1, equalsBlock, 0, 0);
      */

      JUnitTools.assertMatrixEquals("", weight, weightBlock, epsilon);
      JUnitTools.assertMatrixEquals("", equals, equalsBlock, epsilon);
   }

   private void checkRecursionMultiplier(int footstepIndex, double recursionMultiplier)
   {
      DenseMatrix64F multiplier = CommonOps.identity(2, 2);
      CommonOps.scale(recursionMultiplier, multiplier);

      JUnitTools.assertMatrixEquals("", multiplier, footstepRecursionMultipliers.get(footstepIndex), epsilon);
   }

   private void checkReferenceFootstepLocation(int footstepIndex, FramePoint2d referenceFootstepLocation)
   {
      DenseMatrix64F location = new DenseMatrix64F(2, 1);
      location.set(0, 0, referenceFootstepLocation.getX());
      location.set(1, 0, referenceFootstepLocation.getY());

      JUnitTools.assertMatrixEquals("", location, referenceFootstepLocations.get(footstepIndex), epsilon);
   }

   private static final ICPOptimizationParameters icpOptimizationParameters = new ICPOptimizationParameters()
   {
      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override
      public int numberOfFootstepsToConsider()
      {
         return 1;
      }

      @Override public double getForwardFootstepWeight()
      {
         return 10.0;
      }

      @Override public double getLateralFootstepWeight()
      {
         return 10.0;
      }

      @Override public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackForwardWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackLateralWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackParallelGain()
      {
         return 2.0;
      }

      @Override public double getFeedbackOrthogonalGain()
      {
         return 2.0;
      }

      @Override public double getDynamicRelaxationWeight()
      {
         return 1000.0;
      }

      @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override public boolean useFeedback()
      {
         return true;
      }

      @Override public boolean useFeedbackRegularization()
      {
         return true;
      }

      @Override public boolean useStepAdjustment()
      {
         return false;
      }

      @Override public boolean useFootstepRegularization()
      {
         return false;
      }

      @Override public boolean useFeedbackWeightHardening()
      {
         return false;
      }

      @Override public boolean useICPFromBeginningOfState()
      {
         return true;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumTimeRemaining()
      {
         return 0.001;
      }

      @Override public double getFeedbackWeightHardeningMultiplier()
      {
         return 1.0;
      }

      @Override public double getDoubleSupportMaxCMPForwardExit()
      {
         return 0.05;
      }

      @Override public double getDoubleSupportMaxCMPLateralExit()
      {
         return 0.03;
      }

      @Override public double getSingleSupportMaxCMPForwardExit()
      {
         return 0.05;
      }

      @Override public double getSingleSupportMaxCMPLateralExit()
      {
         return 0.03;
      }

      @Override public double getAdjustmentDeadband()
      {
         return 0.0;
      }

      @Override public double getRemainingTimeToStopAdjusting()
      {
         return 0.0;
      }
   };

   private static final CapturePointPlannerParameters icpPlannerParameters = new CapturePointPlannerParameters()
   {
      @Override public double getDoubleSupportInitialTransferDuration()
      {
         return 1.0;
      }

      @Override public double getEntryCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getExitCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public boolean useTwoCMPsPerSupport()
      {
         return false;
      }

      @Override public double getMaxEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMaxExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinExitCMPForwardOffset()
      {
         return 0;
      }
   };
}
