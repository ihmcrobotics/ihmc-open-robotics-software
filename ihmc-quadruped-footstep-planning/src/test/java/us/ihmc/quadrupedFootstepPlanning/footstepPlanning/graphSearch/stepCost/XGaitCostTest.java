package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import static us.ihmc.quadrupedPlanning.QuadrupedSpeed.MEDIUM;
import static us.ihmc.robotics.Assert.assertTrue;

public class XGaitCostTest
{
   private static final double epsilon = 1e-8;
   private static final double stepDuration = 0.5;
   private static final double doubleSupportDuration = 0.2;

   @Test
   public void testComputeTimeDeltaBetweenStepsPace()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.PACE.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getPaceMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getPaceMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(),  new ForwardVelocityProvider());

      String errorMessage = "";
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.FRONT_LEFT, xGaitSettings);
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.FRONT_RIGHT, xGaitSettings);

      errorMessage += testTimeDelta(0.0, RobotQuadrant.HIND_RIGHT, xGaitSettings);
      errorMessage += testTimeDelta(0.0, RobotQuadrant.HIND_LEFT, xGaitSettings);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }

   @Test
   public void testComputeTimeDeltaBetweenStepsCrawl()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getAmbleMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getAmbleMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(),  new ForwardVelocityProvider());

      String errorMessage = "";
      errorMessage += testTimeDelta(0.5 * (stepDuration + doubleSupportDuration), RobotQuadrant.FRONT_LEFT, xGaitSettings);
      errorMessage += testTimeDelta(0.5 * (stepDuration + doubleSupportDuration), RobotQuadrant.FRONT_RIGHT, xGaitSettings);

      errorMessage += testTimeDelta(0.5 * (stepDuration + doubleSupportDuration), RobotQuadrant.HIND_RIGHT, xGaitSettings);
      errorMessage += testTimeDelta(0.5 * (stepDuration + doubleSupportDuration), RobotQuadrant.HIND_LEFT, xGaitSettings);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }

   @Test
   public void testComputeTimeDeltaBetweenStepsTrot()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(),  new ForwardVelocityProvider());

      String errorMessage = "";
      errorMessage += testTimeDelta(0.0, RobotQuadrant.FRONT_LEFT, xGaitSettings);
      errorMessage += testTimeDelta(0.0, RobotQuadrant.FRONT_RIGHT, xGaitSettings);

      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.HIND_RIGHT, xGaitSettings);
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.HIND_LEFT, xGaitSettings);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }


   @Test
   public void testForwardCost()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.5);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(), new ForwardVelocityProvider());
      ReferenceFrame yawedFrame = new PoseReferenceFrame("yawedFrame", ReferenceFrame.getWorldFrame());
      ((PoseReferenceFrame) yawedFrame).setOrientationAndUpdate(new Quaternion(Math.toRadians(45.0), 0.0, 0.0));

      FramePoint2D hindLeft = new FramePoint2D(yawedFrame, -0.5, 0.25);
      FramePoint2D hindRight = new FramePoint2D(yawedFrame, -0.5, -0.25);
      FramePoint2D frontLeft = new FramePoint2D(yawedFrame, 0.5, 0.25);
      FramePoint2D frontRight = new FramePoint2D(yawedFrame, 0.5, -0.25);

      FramePoint2D nextFrontLeft = new FramePoint2D(yawedFrame, 0.8, -0.25);

      hindLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      hindRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      nextFrontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      FootstepNode startNode = new FootstepNode(RobotQuadrant.FRONT_LEFT.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight, 1.0, 0.5);
      FootstepNode endNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, nextFrontLeft, frontRight, hindLeft, hindRight, 1.0, 0.5);

      xGaitCost.compute(startNode, endNode);
   }

   private String testTimeDelta(double expectedDuration, RobotQuadrant robotQuadrant, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      String message = "";
      double actual = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(robotQuadrant, xGaitSettings);
      if (!MathTools.epsilonEquals(expectedDuration, actual, epsilon))
         message += "\n" + robotQuadrant + " expected duration " + expectedDuration + ", got " + actual;

      return message;
   }

   private class TestSnapper extends FootstepNodeSnapper
   {
      @Override
      protected FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
      {
         return FootstepNodeSnapData.identityData();
      }
   }
}
