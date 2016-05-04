package us.ihmc.quadrupedRobotics.planning;

import org.junit.Test;
import static org.junit.Assert.*;
import static us.ihmc.tools.testing.TestPlanTarget.*;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.testing.TestPlanAnnotations;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

@TestPlanAnnotations.DeployableTestClass(targets = Fast)
public class QuadrupedXGaitPlannerTest
{
   @TestPlanAnnotations.DeployableTestMethod
   @Test(timeout=300000)
   public void testForwardVelocityPlan()
   {
      QuadrupedXGaitPlanner xGaitPlanner = new QuadrupedXGaitPlanner();
      QuadrupedXGaitSettings xGaitSettings = xGaitPlanner.getXGaitSettings();
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.25);
      xGaitSettings.setStepGroundClearance(0.1);
      xGaitSettings.setStepDuration(0.25);
      xGaitSettings.setEndPairSupportDuration(0);
      xGaitSettings.setEndPhaseShift(90);

      Vector3d planarVelocity = new Vector3d(1.0, 0.0, 0.0);
      RobotQuadrant initialStepQuadrant = RobotQuadrant.HIND_RIGHT;
      FramePoint initialStepGoalPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      initialStepGoalPosition.set(0.0, 0.0, 0.0);
      double initialStepStartTime = 0.0;
      double initialYaw = 0.0;

      ArrayList<QuadrupedTimedStep> plannedSteps = new ArrayList<>();
      for (int i = 0; i < 4; i++)
      {
         plannedSteps.add(new QuadrupedTimedStep());
      }

      xGaitPlanner.plan(plannedSteps, planarVelocity, initialStepQuadrant, initialStepGoalPosition, initialStepStartTime, initialYaw);

      ArrayList<QuadrupedTimedStep> nominalSteps = new ArrayList<>();
      for (int i = 0; i < 4; i++)
      {
         nominalSteps.add(new QuadrupedTimedStep());
      }

      nominalSteps.get(0).setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
      nominalSteps.get(0).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(0).getGoalPosition().set(0.0, 0.0, 0.0);
      nominalSteps.get(0).getTimeInterval().setInterval(0.0, 0.25);

      nominalSteps.get(1).setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      nominalSteps.get(1).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(1).getGoalPosition().set(1.125, 0.0, 0.0);
      nominalSteps.get(1).getTimeInterval().setInterval(0.125, 0.375);

      nominalSteps.get(2).setRobotQuadrant(RobotQuadrant.HIND_LEFT);
      nominalSteps.get(2).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(2).getGoalPosition().set(0.25, 0.25, 0.0);
      nominalSteps.get(2).getTimeInterval().setInterval(0.25, 0.5);

      nominalSteps.get(3).setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      nominalSteps.get(3).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(3).getGoalPosition().set(1.375, 0.25, 0.0);
      nominalSteps.get(3).getTimeInterval().setInterval(0.375, 0.625);

      double epsilon = 0.00001;
      for (int i = 0; i < 4; i++)
      {
         assertTrue("planned step " + i  + " does not match nominal step", plannedSteps.get(i).epsilonEquals(nominalSteps.get(i), epsilon));
      }
   }
}