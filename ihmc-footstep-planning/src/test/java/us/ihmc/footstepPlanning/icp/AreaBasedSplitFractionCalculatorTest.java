package us.ihmc.footstepPlanning.icp;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.robotics.Assert.assertEquals;

public class AreaBasedSplitFractionCalculatorTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testShiftingFromArea()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      SplitFractionCalculatorParametersBasics parameters = new DefaultSplitFractionCalculatorParameters();

      double fractionLoadIfOtherFootHasNoWidth = 0.5;
      double splitFractionIfOtherFootHasNoWidth = 0.5;
      double fractionLoadIfFootHasFullSupport = 0.8;
      double splitFractionIfFootHasFullSupport = 0.9;
      parameters.setFractionLoadIfOtherFootHasNoWidth(fractionLoadIfOtherFootHasNoWidth);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(splitFractionIfOtherFootHasNoWidth);
      parameters.setFractionLoadIfFootHasFullSupport(fractionLoadIfFootHasFullSupport);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(splitFractionIfFootHasFullSupport);

      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      AreaBasedSplitFractionCalculator areaBasedSplitFractionCalculator = new AreaBasedSplitFractionCalculator(parameters, footPolygons);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.getStartFootPoses().get(RobotSide.LEFT).getPosition().set(0.0, width / 2.0, 0.0);
      request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().set(0.0, -width / 2.0, 0.0);
      request.getStartFootholds().get(RobotSide.LEFT).set(getFullSupportFoot());
      request.getStartFootholds().get(RobotSide.RIGHT).set(getFullSupportFoot());

      FootstepPlan footstepPlan = new FootstepPlan();

      PlannedFootstep firstStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D firstStepPose = new Pose3D(stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      firstStep.getFootstepPose().set(firstStepPose);
      firstStep.getFoothold().set(getLineSupportFoot());
      footstepPlan.addFootstep(firstStep);

      PlannedFootstep secondStep = new PlannedFootstep(stanceSide);
      Pose3D secondStepPose = new Pose3D(2.0 * stepLength, stanceSide.negateIfRightSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      secondStep.getFootstepPose().set(secondStepPose);
      secondStep.getFoothold().set(getLineSupportFoot());
      footstepPlan.addFootstep(secondStep);

      PlannedFootstep thirdStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D thirdStepPose = new Pose3D(2.0 * stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      thirdStep.getFootstepPose().set(thirdStepPose);
      thirdStep.getFoothold().set(getFullSupportFoot());
      footstepPlan.addFootstep(thirdStep);

      double expectedWeightShiftFromUnequal = ((fractionLoadIfFootHasFullSupport - 0.5) + (fractionLoadIfOtherFootHasNoWidth - 0.5)) / 2.0;
      double expectedSplitFractionFromUnequal = ((splitFractionIfFootHasFullSupport - 0.5) + (splitFractionIfOtherFootHasNoWidth - 0.5)) / 2.0;

      areaBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(3, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(secondStepPose.getPosition(), footstepPlan.getFootstep(1).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(secondStepPose.getOrientation(), footstepPlan.getFootstep(1).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(thirdStepPose.getPosition(), footstepPlan.getFootstep(2).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(thirdStepPose.getOrientation(), footstepPlan.getFootstep(2).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      assertEquals(0.5 - expectedWeightShiftFromUnequal, secondStep.getTransferWeightDistribution());
      assertEquals(0.5 + expectedSplitFractionFromUnequal, secondStep.getTransferSplitFraction());

      assertEquals(-1.0, thirdStep.getTransferWeightDistribution());
      assertEquals(-1.0, thirdStep.getTransferSplitFraction());

      assertEquals(0.5 + expectedWeightShiftFromUnequal, footstepPlan.getFinalTransferWeightDistribution());
      assertEquals(0.5 - expectedSplitFractionFromUnequal, footstepPlan.getFinalTransferSplitFraction());
   }

   @Test
   public void testShiftingFromWidth()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      SplitFractionCalculatorParametersBasics parameters = new DefaultSplitFractionCalculatorParameters();

      double fractionLoadIfOtherFootHasNoWidth = 0.8;
      double splitFractionIfOtherFootHasNoWidth = 0.9;
      double fractionLoadIfFootHasFullSupport = 0.5;
      double splitFractionIfFootHasFullSupport = 0.5;
      parameters.setFractionLoadIfOtherFootHasNoWidth(fractionLoadIfOtherFootHasNoWidth);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(splitFractionIfOtherFootHasNoWidth);
      parameters.setFractionLoadIfFootHasFullSupport(fractionLoadIfFootHasFullSupport);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(splitFractionIfFootHasFullSupport);

      AreaBasedSplitFractionCalculator areaBasedSplitFractionCalculator = new AreaBasedSplitFractionCalculator(parameters, PlannerTools.createDefaultFootPolygons());

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.getStartFootPoses().get(RobotSide.LEFT).getPosition().set(0.0, width / 2.0, 0.0);
      request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().set(0.0, -width / 2.0, 0.0);
      request.getStartFootholds().get(RobotSide.LEFT).set(getFullSupportFoot());
      request.getStartFootholds().get(RobotSide.RIGHT).set(getFullSupportFoot());

      FootstepPlan footstepPlan = new FootstepPlan();

      PlannedFootstep firstStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D firstStepPose = new Pose3D(stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      firstStep.getFootstepPose().set(firstStepPose);
      firstStep.getFoothold().set(getLineSupportFoot());
      footstepPlan.addFootstep(firstStep);

      PlannedFootstep secondStep = new PlannedFootstep(stanceSide);
      Pose3D secondStepPose = new Pose3D(2.0 * stepLength, stanceSide.negateIfRightSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      secondStep.getFootstepPose().set(secondStepPose);
      secondStep.getFoothold().set(getLineSupportFoot());
      footstepPlan.addFootstep(secondStep);

      PlannedFootstep thirdStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D thirdStepPose = new Pose3D(2.0 * stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      thirdStep.getFootstepPose().set(thirdStepPose);
      thirdStep.getFoothold().set(getFullSupportFoot());
      footstepPlan.addFootstep(thirdStep);

      double expectedWeightShiftFromUnequal = ((fractionLoadIfFootHasFullSupport - 0.5) + (fractionLoadIfOtherFootHasNoWidth - 0.5)) / 2.0;
      double expectedSplitFractionFromUnequal = ((splitFractionIfFootHasFullSupport - 0.5) + (splitFractionIfOtherFootHasNoWidth - 0.5)) / 2.0;

      areaBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(3, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(secondStepPose.getPosition(), footstepPlan.getFootstep(1).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(secondStepPose.getOrientation(), footstepPlan.getFootstep(1).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(thirdStepPose.getPosition(), footstepPlan.getFootstep(2).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(thirdStepPose.getOrientation(), footstepPlan.getFootstep(2).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      assertEquals(0.5 - expectedWeightShiftFromUnequal, secondStep.getTransferWeightDistribution());
      assertEquals(0.5 + expectedSplitFractionFromUnequal, secondStep.getTransferSplitFraction());

      assertEquals(-1.0, thirdStep.getTransferWeightDistribution());
      assertEquals(-1.0, thirdStep.getTransferSplitFraction());

      assertEquals(0.5 + expectedWeightShiftFromUnequal, footstepPlan.getFinalTransferWeightDistribution());
      assertEquals(0.5 - expectedSplitFractionFromUnequal, footstepPlan.getFinalTransferSplitFraction());
   }

   @Test
   public void testShiftingFromWidthAndArea()
   {
      double width = 0.25;
      double stepLength = 0.3;
      RobotSide stanceSide = RobotSide.LEFT;
      SplitFractionCalculatorParametersBasics parameters = new DefaultSplitFractionCalculatorParameters();

      double fractionLoadIfOtherFootHasNoWidth = 0.8;
      double splitFractionIfOtherFootHasNoWidth = 0.9;
      double fractionLoadIfFootHasFullSupport = 0.75;
      double splitFractionIfFootHasFullSupport = 0.85;
      parameters.setFractionLoadIfOtherFootHasNoWidth(fractionLoadIfOtherFootHasNoWidth);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(splitFractionIfOtherFootHasNoWidth);
      parameters.setFractionLoadIfFootHasFullSupport(fractionLoadIfFootHasFullSupport);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(splitFractionIfFootHasFullSupport);

      AreaBasedSplitFractionCalculator areaBasedSplitFractionCalculator = new AreaBasedSplitFractionCalculator(parameters, PlannerTools.createDefaultFootPolygons());

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.getStartFootPoses().get(RobotSide.LEFT).getPosition().set(0.0, width / 2.0, 0.0);
      request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().set(0.0, -width / 2.0, 0.0);
      request.getStartFootholds().get(RobotSide.LEFT).set(getFullSupportFoot());
      request.getStartFootholds().get(RobotSide.RIGHT).set(getFullSupportFoot());

      FootstepPlan footstepPlan = new FootstepPlan();

      PlannedFootstep firstStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D firstStepPose = new Pose3D(stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      firstStep.getFootstepPose().set(firstStepPose);
      firstStep.getFoothold().set(getLineSupportFoot());
      footstepPlan.addFootstep(firstStep);

      PlannedFootstep secondStep = new PlannedFootstep(stanceSide);
      Pose3D secondStepPose = new Pose3D(2.0 * stepLength, stanceSide.negateIfRightSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      secondStep.getFootstepPose().set(secondStepPose);
      secondStep.getFoothold().set(getLineSupportFoot());
      footstepPlan.addFootstep(secondStep);

      PlannedFootstep thirdStep = new PlannedFootstep(stanceSide.getOppositeSide());
      Pose3D thirdStepPose = new Pose3D(2.0 * stepLength, stanceSide.negateIfLeftSide(width / 2.0), 0.0, 0.0, 0.0, 0.0);
      thirdStep.getFootstepPose().set(thirdStepPose);
      thirdStep.getFoothold().set(getFullSupportFoot());
      footstepPlan.addFootstep(thirdStep);

      double expectedWeightShiftFromUnequal = ((fractionLoadIfFootHasFullSupport - 0.5) + (fractionLoadIfOtherFootHasNoWidth - 0.5)) / 2.0;
      double expectedSplitFractionFromUnequal = ((splitFractionIfFootHasFullSupport - 0.5) + (splitFractionIfOtherFootHasNoWidth - 0.5)) / 2.0;

      areaBasedSplitFractionCalculator.computeSplitFractions(request, footstepPlan);
      assertEquals(3, footstepPlan.getNumberOfSteps());

      // check that step poses have not changed
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(firstStepPose.getPosition(), footstepPlan.getFootstep(0).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(firstStepPose.getOrientation(), footstepPlan.getFootstep(0).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(secondStepPose.getPosition(), footstepPlan.getFootstep(1).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(secondStepPose.getOrientation(), footstepPlan.getFootstep(1).getFootstepPose().getOrientation(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(thirdStepPose.getPosition(), footstepPlan.getFootstep(2).getFootstepPose().getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(thirdStepPose.getOrientation(), footstepPlan.getFootstep(2).getFootstepPose().getOrientation(), epsilon);

      // check split fractions
      assertEquals(-1.0, firstStep.getTransferSplitFraction());
      assertEquals(-1.0, firstStep.getTransferWeightDistribution());

      assertEquals(0.5 - expectedWeightShiftFromUnequal, secondStep.getTransferWeightDistribution());
      assertEquals(0.5 + expectedSplitFractionFromUnequal, secondStep.getTransferSplitFraction());

      assertEquals(-1.0, thirdStep.getTransferWeightDistribution());
      assertEquals(-1.0, thirdStep.getTransferSplitFraction());

      assertEquals(0.5 + expectedWeightShiftFromUnequal, footstepPlan.getFinalTransferWeightDistribution());
      assertEquals(0.5 - expectedSplitFractionFromUnequal, footstepPlan.getFinalTransferSplitFraction());
   }

   private static ConvexPolygon2D getFullSupportFoot()
   {
      double length = 0.3;
      double width = 0.15;
      return PlannerTools.createFootPolygon(length, width);
   }

   private static ConvexPolygon2D getLineSupportFoot()
   {
      double length = 0.3;
      double width = 0.0;
      return PlannerTools.createFootPolygon(length, width);
   }
}
