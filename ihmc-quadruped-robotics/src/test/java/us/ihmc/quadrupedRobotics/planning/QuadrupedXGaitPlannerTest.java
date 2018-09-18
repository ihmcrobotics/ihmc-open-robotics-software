package us.ihmc.quadrupedRobotics.planning;

import org.junit.After;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.planning.bodyPath.QuadrupedPlanarBodyPathProvider;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedPlanarFootstepPlan;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertTrue;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class QuadrupedXGaitPlannerTest
{
   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testInitialForwardVelocityPlan()
   {
      ForwardMotionBodyPathProvider bodyPathProvider = new ForwardMotionBodyPathProvider();
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.25);
      xGaitSettings.setStepGroundClearance(0.1);
      xGaitSettings.setStepDuration(0.25);
      xGaitSettings.setEndDoubleSupportDuration(0);
      xGaitSettings.setEndPhaseShift(90);
      QuadrupedXGaitPlanner xGaitPlanner = new QuadrupedXGaitPlanner(bodyPathProvider, xGaitSettings);

      xGaitPlanner.setStepSnapper((x, y) -> new Point3D(x, y, 0.0));



      RobotQuadrant initialStepQuadrant = RobotQuadrant.HIND_RIGHT;
      FramePoint3D supportCentroidAtSoS = new FramePoint3D(ReferenceFrame.getWorldFrame());
      supportCentroidAtSoS.set(0, 0, 0);
      double timeAtSoS = 0.0;

      QuadrupedPlanarFootstepPlan footstepPlan = new QuadrupedPlanarFootstepPlan(4);
      bodyPathProvider.initialPose.setToZero(ReferenceFrame.getWorldFrame());
      bodyPathProvider.desiredForwardMotion = 1.0;
      xGaitPlanner.computeInitialPlan(footstepPlan, initialStepQuadrant, timeAtSoS);

      ArrayList<QuadrupedTimedOrientedStep> nominalSteps = new ArrayList<>();
      for (int i = 0; i < 4; i++)
      {
         nominalSteps.add(new QuadrupedTimedOrientedStep());
      }

      nominalSteps.get(0).setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
      nominalSteps.get(0).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(0).setGoalPosition(new Point3D(-0.25, -0.125, 0.0));
      nominalSteps.get(0).getTimeInterval().setInterval(0.0, 0.25);
      nominalSteps.get(0).setStepYaw(0.0);

      nominalSteps.get(1).setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      nominalSteps.get(1).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(1).setGoalPosition(new Point3D(0.875, -0.125, 0.0));
      nominalSteps.get(1).getTimeInterval().setInterval(0.125, 0.375);
      nominalSteps.get(1).setStepYaw(0.0);

      nominalSteps.get(2).setRobotQuadrant(RobotQuadrant.HIND_LEFT);
      nominalSteps.get(2).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(2).setGoalPosition(new Point3D(0.0, 0.125, 0.0));
      nominalSteps.get(2).getTimeInterval().setInterval(0.25, 0.5);
      nominalSteps.get(2).setStepYaw(0.0);

      nominalSteps.get(3).setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      nominalSteps.get(3).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(3).setGoalPosition(new Point3D(1.125, 0.125, 0.0));
      nominalSteps.get(3).getTimeInterval().setInterval(0.375, 0.625);
      nominalSteps.get(3).setStepYaw(0.0);

      double epsilon = 0.00001;
      List<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
      for (int i = 0; i < 4; i++)
      {
         assertTrue("planned step " + i + " does not match nominal step", plannedSteps.get(i).epsilonEquals(nominalSteps.get(i), epsilon));
      }
   }

      @ContinuousIntegrationTest(estimatedDuration = 0.0)
      @Test(timeout=300000)
      public void testOnlineForwardVelocityPlan()
      {
         ForwardMotionBodyPathProvider bodyPathProvider = new ForwardMotionBodyPathProvider();
         QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
         xGaitSettings.setStanceLength(1.0);
         xGaitSettings.setStanceWidth(0.25);
         xGaitSettings.setStepGroundClearance(0.1);
         xGaitSettings.setStepDuration(0.25);
         xGaitSettings.setEndDoubleSupportDuration(0);
         xGaitSettings.setEndPhaseShift(90);
         QuadrupedXGaitPlanner xGaitPlanner = new QuadrupedXGaitPlanner(bodyPathProvider, xGaitSettings);
         xGaitPlanner.setStepSnapper((x, y) -> new Point3D(x, y, 0.0));

         double currentTime = 0.125;
         double currentYaw = 0.0;
         double currentHeight = 0.0;
         QuadrupedPlanarFootstepPlan footstepPlan = new QuadrupedPlanarFootstepPlan(4);

         QuadrupedTimedOrientedStep currentHindStep = new QuadrupedTimedOrientedStep();
         currentHindStep.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
         currentHindStep.setGroundClearance(xGaitSettings.getStepGroundClearance());
         currentHindStep.setGoalPosition(new Point3D(-0.25, -0.125, 0.0));
         currentHindStep.getTimeInterval().setInterval(0.0, 0.25);
         currentHindStep.setStepYaw(0.0);
         currentHindStep.setGroundClearance(currentHeight);
         footstepPlan.getCurrentSteps().set(RobotEnd.HIND, currentHindStep);

         QuadrupedTimedOrientedStep currentFrontStep = new QuadrupedTimedOrientedStep();
         currentFrontStep.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
         currentFrontStep.setGroundClearance(xGaitSettings.getStepGroundClearance());
         currentFrontStep.setGoalPosition(new Point3D(0.875, -0.125, 0.0));
         currentFrontStep.getTimeInterval().setInterval(0.125, 0.375);
         currentFrontStep.setStepYaw(0.0);
         currentHindStep.setGroundClearance(currentHeight);
         footstepPlan.getCurrentSteps().set(RobotEnd.FRONT, currentFrontStep);

         bodyPathProvider.initialPose.setToZero(ReferenceFrame.getWorldFrame());
         bodyPathProvider.desiredForwardMotion = 1.0;
         xGaitPlanner.computeOnlinePlan(footstepPlan, currentTime);

         ArrayList<QuadrupedTimedStep> nominalSteps = new ArrayList<>();
         for (int i = 0; i < 2; i++)
         {
            nominalSteps.add(new QuadrupedTimedStep());
         }

         nominalSteps.get(0).setRobotQuadrant(RobotQuadrant.HIND_LEFT);
         nominalSteps.get(0).setGroundClearance(xGaitSettings.getStepGroundClearance());
         nominalSteps.get(0).setGoalPosition(new Point3D(0.0, 0.125, 0.0));
         nominalSteps.get(0).getTimeInterval().setInterval(0.25, 0.5);

         nominalSteps.get(1).setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
         nominalSteps.get(1).setGroundClearance(xGaitSettings.getStepGroundClearance());
         nominalSteps.get(1).setGoalPosition(new Point3D(1.125, 0.125, 0.0));
         nominalSteps.get(1).getTimeInterval().setInterval(0.375, 0.625);

         double epsilon = 0.00001;
         List<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
         for (int i = 0; i < 2; i++)
         {
            assertTrue("planned step " + i  + " does not match nominal step", plannedSteps.get(i).epsilonEquals(nominalSteps.get(i), epsilon));
         }
      }

   private class ForwardMotionBodyPathProvider implements QuadrupedPlanarBodyPathProvider
   {
      double desiredForwardMotion;
      FramePose2D initialPose = new FramePose2D();

      @Override
      public void initialize()
      {
      }

      @Override
      public void getPlanarPose(double time, FramePose2D poseToPack)
      {
         double xDisplacement = desiredForwardMotion * time;
         poseToPack.setX(initialPose.getX() + xDisplacement);
      }
   }
}