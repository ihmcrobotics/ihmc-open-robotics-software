package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
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
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class XGaitCostTest
{
   private static final double epsilon = 1e-8;
   private static final double stepDuration = 0.5;
   private static final double doubleSupportDuration = 0.2;
   private static final double maxSpeed = 0.3;
   private static final double planningSpeedFraction = 0.8;

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
   public void testForwardWithFrontEndCost()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);
      xGaitSettings.getTrotMediumTimings().setMaxSpeed(maxSpeed);
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.5);

      FootstepPlannerParameters footstepPlannerParameters = new TestParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(), new ForwardVelocityProvider());
      ReferenceFrame yawedFrame = new PoseReferenceFrame("yawedFrame", ReferenceFrame.getWorldFrame());
      ((PoseReferenceFrame) yawedFrame).setOrientationAndUpdate(new Quaternion(Math.toRadians(45.0), 0.0, 0.0));

      FramePoint2D hindLeft = new FramePoint2D(yawedFrame, -0.5, 0.25);
      FramePoint2D hindRight = new FramePoint2D(yawedFrame, -0.5, -0.25);
      FramePoint2D frontLeft = new FramePoint2D(yawedFrame, 0.5, 0.25);
      FramePoint2D frontRight = new FramePoint2D(yawedFrame, 0.5, -0.25);

      double forwardVelocity = maxSpeed * planningSpeedFraction;
      double forwardDisplacement = forwardVelocity * (stepDuration + doubleSupportDuration);

      FramePoint2D nextFrontLeft = new FramePoint2D(yawedFrame, 0.5 + forwardDisplacement, 0.25);

      hindLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      hindRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      nextFrontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      RobotQuadrant steppingQuadrant = RobotQuadrant.FRONT_LEFT;
      FootstepNode startNode = new FootstepNode(steppingQuadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight,
                                                1.0, 0.5);
      FootstepNode endNode = new FootstepNode(steppingQuadrant, nextFrontLeft, frontRight, hindLeft, hindRight, 1.0, 0.5);

      assertEquals(0.0,  xGaitCost.compute(startNode, endNode), 5e-2);


      FramePoint2D nextFrontRight = new FramePoint2D(yawedFrame, 0.5 + forwardDisplacement, -0.25);
      nextFrontRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      steppingQuadrant = RobotQuadrant.FRONT_RIGHT;
      startNode = new FootstepNode(steppingQuadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight,
                                            1.0, 0.5);
      endNode = new FootstepNode(steppingQuadrant, frontLeft, nextFrontRight, hindLeft, hindRight, 1.0, 0.5);

      assertEquals(0.0,  xGaitCost.compute(startNode, endNode), 5e-2);
   }

   @Test
   public void testForwardWithHindEndCost()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);
      xGaitSettings.getTrotMediumTimings().setMaxSpeed(0.3);
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.5);

      FootstepPlannerParameters footstepPlannerParameters = new TestParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(), new ForwardVelocityProvider());
      ReferenceFrame yawedFrame = new PoseReferenceFrame("yawedFrame", ReferenceFrame.getWorldFrame());
      double yaw = Math.toRadians(45.0);
      ((PoseReferenceFrame) yawedFrame).setOrientationAndUpdate(new Quaternion(yaw, 0.0, 0.0));

      FramePoint2D hindLeft = new FramePoint2D(yawedFrame, -0.5, 0.25);
      FramePoint2D hindRight = new FramePoint2D(yawedFrame, -0.5, -0.25);
      FramePoint2D frontLeft = new FramePoint2D(yawedFrame, 0.75, 0.25);
      FramePoint2D frontRight = new FramePoint2D(yawedFrame, 0.5, -0.25);

      FramePoint2D nextHindRight = new FramePoint2D(yawedFrame, -0.25, -0.25);

      hindLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      hindRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      nextHindRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      RobotQuadrant steppingQuadrant = RobotQuadrant.HIND_RIGHT;
      FootstepNode startNode = new FootstepNode(steppingQuadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight,
                                                1.0, 0.5);
      FootstepNode endNode = new FootstepNode(steppingQuadrant, frontLeft, frontRight, hindLeft, nextHindRight, 1.0, 0.5);

      assertEquals(0.0,  xGaitCost.compute(startNode, endNode), 5e-2);

      frontLeft = new FramePoint2D(yawedFrame, 0.5, 0.25);
      frontRight = new FramePoint2D(yawedFrame, 0.75, -0.25);

      FramePoint2D nextHindLeft = new FramePoint2D(yawedFrame, -0.25, 0.25);

      frontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      nextHindLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      steppingQuadrant = RobotQuadrant.HIND_LEFT;
      startNode = new FootstepNode(steppingQuadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight,
                                                1.0, 0.5);
      endNode = new FootstepNode(steppingQuadrant, frontLeft, frontRight, nextHindLeft, hindRight, 1.0, 0.5);

      assertEquals(0.0,  xGaitCost.compute(startNode, endNode), 5e-2);
   }



   @Test
   public void testBackwardCost()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);
      xGaitSettings.getTrotMediumTimings().setMaxSpeed(0.3);
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.5);

      FootstepPlannerParameters footstepPlannerParameters = new TestParameters();

      StraightShotVelocityProvider velocityProvider = new StraightShotVelocityProvider();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings, new TestSnapper(), velocityProvider);
      ReferenceFrame yawedFrame = new PoseReferenceFrame("yawedFrame", ReferenceFrame.getWorldFrame());
      ((PoseReferenceFrame) yawedFrame).setOrientationAndUpdate(new Quaternion(Math.toRadians(45.0), 0.0, 0.0));

      FramePoint2D hindLeft = new FramePoint2D(yawedFrame, -0.5, 0.25);
      FramePoint2D hindRight = new FramePoint2D(yawedFrame, -0.5, -0.25);
      FramePoint2D frontLeft = new FramePoint2D(yawedFrame, 0.5, 0.25);
      FramePoint2D frontRight = new FramePoint2D(yawedFrame, 0.5, -0.25);

      FramePoint2D hindLeftGoal = new FramePoint2D(yawedFrame, -3.5, 0.25);
      FramePoint2D hindRightGoal = new FramePoint2D(yawedFrame, -3.5, -0.25);
      FramePoint2D frontLeftGoal = new FramePoint2D(yawedFrame, -2.5, 0.25);
      FramePoint2D frontRightGoal = new FramePoint2D(yawedFrame, -2.5, -0.25);

      double forwardVelocity = maxSpeed * planningSpeedFraction;
      double forwardDisplacement = forwardVelocity * (stepDuration + doubleSupportDuration);
      FramePoint2D nextFrontLeft = new FramePoint2D(yawedFrame, 0.5 - forwardDisplacement, 0.25);

      hindLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      hindRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontRight.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      hindLeftGoal.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      hindRightGoal.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontLeftGoal.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      frontRightGoal.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      nextFrontLeft.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      FootstepNode startNode = new FootstepNode(RobotQuadrant.FRONT_LEFT.getNextReversedRegularGaitSwingQuadrant(), frontLeft, frontRight, hindLeft, hindRight, 1.0, 0.5);
      FootstepNode endNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, nextFrontLeft, frontRight, hindLeft, hindRight, 1.0, 0.5);
      FootstepNode goalNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeftGoal, frontRightGoal, hindLeftGoal, hindRightGoal, 1.0, 0.5);

      velocityProvider.setGoalNode(goalNode);

      assertEquals(0.0,  xGaitCost.compute(startNode, endNode), 5e-2);
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

   private class TestParameters extends DefaultFootstepPlannerParameters
   {
      @Override
      public double getXGaitWeight()
      {
         return 1.0;
      }

      @Override
      public double getMaxWalkingSpeedMultiplier()
      {
         return planningSpeedFraction;
      }
   }

}
