package us.ihmc.footstepPlanning.icp;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.robotics.Assert.assertEquals;

public class PositionBasedSplitFractionCalculatorTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testStepDownWithClosingStep()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      SplitFractionCalculatorParametersBasics parameters = new DefaultSplitFractionCalculatorParameters();

      double fullSplitFractionFromHeight = 0.895;
      double fullWeightDistributionFromHeight = 0.8;
      parameters.setTransferSplitFractionAtFullDepth(fullSplitFractionFromHeight);
      parameters.setTransferWeightDistributionAtFullDepth(fullWeightDistributionFromHeight);

      double minHeightToStartShifting = -parameters.getStepHeightForLargeStepDown();
      double heightForFullShift = -parameters.getLargestStepDownHeight();

      PositionBasedSplitFractionCalculator positionBasedSplitFractionCalculator = new PositionBasedSplitFractionCalculator(parameters);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.getStartFootPoses().get(RobotSide.LEFT).getPosition().set(0.0, width / 2.0, 0.0);
      request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().set(0.0, -width / 2.0, 0.0);

      FootstepPlan footstepPlan = new FootstepPlan();

      PlannedFootstep firstStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D firstStepPose = new Pose3D(stepLength, stanceSide.negateIfLeftSide(width / 2.0), minHeightToStartShifting, 0.0, 0.0, 0.0);
      firstStep.getFootstepPose().set(firstStepPose);
      footstepPlan.addFootstep(firstStep);

      PlannedFootstep secondStep = new PlannedFootstep(stanceSide);
      Pose3D secondStepPose = new Pose3D(stepLength, stanceSide.negateIfRightSide(width / 2.0), minHeightToStartShifting, 0.0, 0.0, 0.0);
      secondStep.getFootstepPose().set(secondStepPose);
      footstepPlan.addFootstep(secondStep);

      positionBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(2, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(secondStepPose.getPosition(), footstepPlan.getFootstep(1).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(secondStepPose.getOrientation(), footstepPlan.getFootstep(1).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(-1.0, footstepPlan.getFinalTransferSplitFraction());
      assertEquals(-1.0, footstepPlan.getFinalTransferWeightDistribution());

      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      assertEquals(-1.0, secondStep.getTransferSplitFraction());
      assertEquals(-1.0, secondStep.getTransferWeightDistribution());

      // check for full shift
      firstStepPose.getPosition().setZ(heightForFullShift - 0.05);
      secondStepPose.getPosition().setZ(heightForFullShift - 0.05);
      firstStep.getFootstepPose().set(firstStepPose);
      secondStep.getFootstepPose().set(secondStepPose);

      positionBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(2, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(secondStepPose.getPosition(), footstepPlan.getFootstep(1).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(secondStepPose.getOrientation(), footstepPlan.getFootstep(1).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(-1.0, footstepPlan.getFinalTransferSplitFraction());
      assertEquals(-1.0, footstepPlan.getFinalTransferWeightDistribution());

      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      assertEquals(fullSplitFractionFromHeight, secondStep.getTransferSplitFraction());
      assertEquals(fullWeightDistributionFromHeight, secondStep.getTransferWeightDistribution());

      // check between starting to shift and full shift
      double incrementToCheck = 0.01;
      for (double height = minHeightToStartShifting - incrementToCheck; height >= heightForFullShift; height -= incrementToCheck)
      {
         // reset transfer timing values
         firstStep.setTransferSplitFraction(-1.0);
         firstStep.setTransferWeightDistribution(-1.0);
         secondStep.setTransferSplitFraction(-1.0);
         secondStep.setTransferWeightDistribution(-1.0);

         firstStepPose.getPosition().setZ(height);
         secondStepPose.getPosition().setZ(height);
         firstStep.getFootstepPose().set(firstStepPose);
         secondStep.getFootstepPose().set(secondStepPose);

         positionBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
         assertEquals(2, footstepPlan.getNumberOfSteps());

         // check that step poses have not changed
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(secondStepPose.getPosition(), footstepPlan.getFootstep(1).getFootstepPose().getPosition(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(secondStepPose.getOrientation(), footstepPlan.getFootstep(1).getFootstepPose().getOrientation(), epsilon);

         // check split fractions
         assertEquals(-1.0, footstepPlan.getFinalTransferSplitFraction());
         assertEquals(-1.0, footstepPlan.getFinalTransferWeightDistribution());

         assertEquals(-1.0, firstStep.getTransferSplitFraction());
         assertEquals(-1.0, firstStep.getTransferWeightDistribution());

         double alphaThrough = (minHeightToStartShifting - height) / (minHeightToStartShifting - heightForFullShift);
         double expectedSplitFraction = InterpolationTools.linearInterpolate(parameters.getDefaultTransferSplitFraction(), fullSplitFractionFromHeight, alphaThrough);
         double expectedWeightDistribution = InterpolationTools.linearInterpolate(0.5, fullWeightDistributionFromHeight, alphaThrough);

         assertEquals(expectedSplitFraction, secondStep.getTransferSplitFraction());
         assertEquals(expectedWeightDistribution, secondStep.getTransferWeightDistribution());
      }
   }

   @Test
   public void testSingleLargeStepDown()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      SplitFractionCalculatorParametersBasics parameters = new DefaultSplitFractionCalculatorParameters();

      double fullSplitFractionFromHeight = 0.895;
      double fullWeightDistributionFromHeight = 0.8;
      parameters.setTransferSplitFractionAtFullDepth(fullSplitFractionFromHeight);
      parameters.setTransferWeightDistributionAtFullDepth(fullWeightDistributionFromHeight);

      double minHeightToStartShifting = -parameters.getStepHeightForLargeStepDown();
      double heightForFullShift = -parameters.getLargestStepDownHeight();

      PositionBasedSplitFractionCalculator positionBasedSplitFractionCalculator = new PositionBasedSplitFractionCalculator(parameters);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.getStartFootPoses().get(RobotSide.LEFT).getPosition().set(0.0, width / 2.0, 0.0);
      request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().set(0.0, -width / 2.0, 0.0);

      FootstepPlan footstepPlan = new FootstepPlan();

      PlannedFootstep firstStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D firstStepPose = new Pose3D(stepLength, stanceSide.negateIfLeftSide(width / 2.0), minHeightToStartShifting, 0.0, 0.0, 0.0);
      firstStep.getFootstepPose().set(firstStepPose);
      footstepPlan.addFootstep(firstStep);

      positionBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(1, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(-1.0, footstepPlan.getFinalTransferSplitFraction());
      assertEquals(-1.0, footstepPlan.getFinalTransferWeightDistribution());

      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      // check for full shift
      firstStepPose.getPosition().setZ(heightForFullShift - 0.05);
      firstStep.getFootstepPose().set(firstStepPose);

      positionBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(1, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(fullSplitFractionFromHeight, footstepPlan.getFinalTransferSplitFraction());
      assertEquals(fullWeightDistributionFromHeight, footstepPlan.getFinalTransferWeightDistribution());

      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      // check between starting to shift and full shift
      double incrementToCheck = 0.01;
      for (double height = minHeightToStartShifting - incrementToCheck; height >= heightForFullShift; height -= incrementToCheck)
      {
         // reset transfer timing values
         firstStep.setTransferSplitFraction(-1.0);
         firstStep.setTransferWeightDistribution(-1.0);
         footstepPlan.setFinalTransferWeightDistribution(-1.0);
         footstepPlan.setFinalTransferSplitFraction(-1.0);

         firstStepPose.getPosition().setZ(height);
         firstStep.getFootstepPose().set(firstStepPose);

         positionBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
         assertEquals(1, footstepPlan.getNumberOfSteps());

         // check that step poses have not changed
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

         // check split fractions
         double alphaThrough = (minHeightToStartShifting - height) / (minHeightToStartShifting - heightForFullShift);
         double expectedSplitFraction = InterpolationTools.linearInterpolate(parameters.getDefaultTransferSplitFraction(), fullSplitFractionFromHeight, alphaThrough);
         double expectedWeightDistribution = InterpolationTools.linearInterpolate(0.5, fullWeightDistributionFromHeight, alphaThrough);

         assertEquals(expectedSplitFraction, footstepPlan.getFinalTransferSplitFraction());
         assertEquals(expectedWeightDistribution, footstepPlan.getFinalTransferWeightDistribution());

         assertEquals(-1.0, firstStep.getTransferSplitFraction());
         assertEquals(-1.0, firstStep.getTransferWeightDistribution());
      }
   }
}
