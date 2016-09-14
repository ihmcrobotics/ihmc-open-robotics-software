package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationSolver;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.ArrayList;
import java.util.Random;

public class ICPOptimizationSolverTest extends ICPOptimizationSolver
{
   private static final YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRobert");

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.0001;
   private static final double omega = 3.5;

   public ICPOptimizationSolverTest()
   {
      super(icpOptimizationParameters, 20, rootRegistry);
      rootRegistry.clear();
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testSetFeedbackConditions()
   {
      Random random = new Random();
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         double feedbackWeight = 10.0 * random.nextDouble();
         double feedbackGain = 10.0 * random.nextDouble();

         super.setFeedbackConditions(feedbackWeight, feedbackGain, 1000.0, 3.5);

         checkFeedbackMatrices(feedbackWeight, feedbackGain);
      }
   }

   private void checkFeedbackMatrices(double feedbackWeight, double feedbackGain)
   {
      Assert.assertEquals("", feedbackWeight, this.feedbackWeight.get(0, 0), epsilon);
      Assert.assertEquals("", feedbackWeight, this.feedbackWeight.get(1, 1), epsilon);

      Assert.assertEquals("", feedbackGain, this.feedbackGain.get(1, 1), epsilon);
      Assert.assertEquals("", feedbackGain, this.feedbackGain.get(1, 1), epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testDimensions()
   {
      for (int i = 1; i < this.maximumNumberOfFootstepsToConsider; i++)
      {
         testDimension(i, 4, false, true, false, false);
         testDimension(i, 4, true, true, false, false);
         testDimension(i, 4, true, false, false, false);
      }
   }

   public void testDimension(int numberOfFootstepsToConsider, int numberOfVertices, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs,
         boolean useActiveCMPOptimization)
   {
      super.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs); //, useActiveCMPOptimization);

      double numberOfLagrangeMultipliers = 2;
      double numberOfFootstepVariables = 0;
      if (useStepAdjustment)
         numberOfFootstepVariables = 2 * numberOfFootstepsToConsider;

      double totalNumberOfFreeVariables = numberOfFootstepVariables;

      if (useFeedback)
         totalNumberOfFreeVariables += 2;

      String name = "Number of Steps: " + numberOfFootstepsToConsider + ". Use step adjustment: " + useStepAdjustment + ". Use Feedback: " + useFeedback;
      Assert.assertEquals(name, numberOfFootstepVariables, this.numberOfFootstepVariables, epsilon);
      Assert.assertEquals(name, numberOfLagrangeMultipliers, this.numberOfLagrangeMultipliers, epsilon);
      Assert.assertEquals(name, totalNumberOfFreeVariables, this.numberOfFreeVariables, epsilon);

      Assert.assertEquals(name, totalNumberOfFreeVariables, solverInput_H.numRows, epsilon);
      Assert.assertEquals(name, totalNumberOfFreeVariables, solverInput_H.numCols, epsilon);
      Assert.assertEquals(name, totalNumberOfFreeVariables, solverInput_h.numRows, epsilon);
      Assert.assertEquals(name, 1, solverInput_h.numCols, epsilon);

      Assert.assertEquals(name, numberOfFootstepVariables, footstepCost_H.numRows, epsilon);
      Assert.assertEquals(name, numberOfFootstepVariables, footstepCost_H.numCols, epsilon);
      Assert.assertEquals(name, numberOfFootstepVariables, footstepCost_h.numRows, epsilon);
      Assert.assertEquals(name, 1, footstepCost_h.numCols, epsilon);

      Assert.assertEquals(name, 2, feedbackCost_H.numRows, epsilon);
      Assert.assertEquals(name, 2, feedbackCost_H.numCols, epsilon);
      Assert.assertEquals(name, 2, feedbackCost_h.numRows, epsilon);
      Assert.assertEquals(name, 1, feedbackCost_h.numCols, epsilon);

      Assert.assertEquals(name, totalNumberOfFreeVariables, solverInput_Aeq.numRows, epsilon);
      Assert.assertEquals(name, numberOfLagrangeMultipliers, solverInput_Aeq.numCols, epsilon);
      Assert.assertEquals(name, numberOfLagrangeMultipliers, solverInput_AeqTrans.numRows, epsilon);
      Assert.assertEquals(name, totalNumberOfFreeVariables, solverInput_AeqTrans.numCols, epsilon);
      Assert.assertEquals(name, numberOfLagrangeMultipliers, solverInput_beq.numRows, epsilon);
      Assert.assertEquals(name, 1, solverInput_beq.numCols, epsilon);

      Assert.assertEquals(name, totalNumberOfFreeVariables, dynamics_Aeq.numRows, epsilon);
      Assert.assertEquals(name, 2, dynamics_Aeq.numCols, epsilon);
      Assert.assertEquals(name, 2, dynamics_beq.numRows, epsilon);
      Assert.assertEquals(name, 1, dynamics_beq.numCols, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testConditionError()
   {
      testCondition(0, 4, true, false, true, false);
      testCondition(0, 4, true, false, false, false);
      testCondition(0, 4, false, false, true, false);
      testCondition(0, 4, false, false, false, false);
      testCondition(1, 4, false, false, true, false);
      testCondition(1, 4, false, false, false, false);
   }

   public void testCondition(int numberOfFootstepsToConsider, int numberOfVertices, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs,
         boolean useActiveCMPOptimization)
   {
      boolean hasError = false;
      try
      {
         super.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs); //, useActiveCMPOptimization);
      }
      catch (RuntimeException e)
      {
         hasError = true;
      }

      Assert.assertTrue(hasError);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDynamicConstraint()
   {
      double omega = 3.0;
      double remainingTime = 0.5;

      double doubleSupportDuration = 0.2;
      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      int numberOffootstepsToConsider = 0;
      super.submitProblemConditions(numberOffootstepsToConsider, true, true, false); //, false);
      super.setFeedbackConditions(2.0, 0.001, 1000.0, omega);

      double finalICPRecursionMultiplier = Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d finalICP = new FramePoint2d(worldFrame, 0.2, 0.115);
      FramePoint2d finalICPRecursion = new FramePoint2d();
      finalICPRecursion.set(finalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      double currentStateProjection = Math.exp(omega * remainingTime);
      double cmpProjectionMultiplier = currentStateProjection - Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.0, -0.155);
      FramePoint2d cmpProjection = new FramePoint2d();
      cmpProjection.set(perfectCMP);
      cmpProjection.scale(cmpProjectionMultiplier);

      FramePoint2d currentICP = new FramePoint2d(worldFrame, 0.1, 0.06);

      compute(finalICPRecursion, null, currentICP, perfectCMP, cmpProjection, currentStateProjection);

      FramePoint2d rightHandSide = new FramePoint2d();
      rightHandSide.set(currentICP);
      rightHandSide.scale(currentStateProjection);
      rightHandSide.sub(cmpProjection);
      rightHandSide.sub(finalICPRecursion);

      DenseMatrix64F expectedDynamics_beq = new DenseMatrix64F(2, 1);
      expectedDynamics_beq.set(0, 0, rightHandSide.getX());
      expectedDynamics_beq.set(1, 0, rightHandSide.getY());

      JUnitTools.assertMatrixEquals(expectedDynamics_beq, dynamics_beq, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
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
         super.setFeedbackConditions(feedbackWeight, feedbackGain, 1000.0, omega);

         checkFeedbackMatrices(feedbackWeight, feedbackGain);
         testDimension(numberOfFootstepsToConsider, numberOfVertices, true, true, false, false);

         super.addFeedbackTask();

         DenseMatrix64F weightBlock = new DenseMatrix64F(2, 2);
         CommonOps.extract(solverInput_H, feedbackIndex, feedbackIndex + 2, feedbackIndex, feedbackIndex + 2, weightBlock, 0, 0);

         String name = "Step number " + numberOfFootstepsToConsider;
         JUnitTools.assertMatrixEquals(name, feedbackCost_H, weightBlock, epsilon);
      }
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
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
         testDimension(numberOfFootstepsToConsider, numberOfVertices, true, true, false, false);

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

         JUnitTools.assertMatrixEquals(footstepCost_H, footstepWeightBlock, epsilon);
         JUnitTools.assertMatrixEquals(footstepCost_h, footstepEqualsBlock, epsilon);
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

      CommonOps.extract(footstepCost_H, index, index + 2, index, index + 2, weightBlock, 0, 0);
      CommonOps.extract(footstepCost_h, index, index + 2, 0, 1, equalsBlock, 0, 0);

      JUnitTools.assertMatrixEquals("", weight, weightBlock, epsilon);
      JUnitTools.assertMatrixEquals("", equals, equalsBlock, epsilon);
   }

   private void checkRecursionMultiplier(int footstepIndex, double recursionMultiplier)
   {
      DenseMatrix64F multiplier = CommonOps.identity(2, 2);
      CommonOps.scale(recursionMultiplier, multiplier);

      JUnitTools.assertMatrixEquals("", multiplier, footstepRecursionMutlipliers.get(footstepIndex), epsilon);
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

      @Override public double getFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackWeight()
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

      @Override public double getMaxCMPExitForward()
      {
         return 0.05;
      }

      @Override public double getMaxCMPExitSideways()
      {
         return 0.03;
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
