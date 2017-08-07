package us.ihmc.quadrupedRobotics.planning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class QuadrupedXGaitPlannerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testInitialForwardVelocityPlan()
   {
      QuadrupedXGaitPlanner xGaitPlanner = new QuadrupedXGaitPlanner();
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.25);
      xGaitSettings.setStepGroundClearance(0.1);
      xGaitSettings.setStepDuration(0.25);
      xGaitSettings.setEndDoubleSupportDuration(0);
      xGaitSettings.setEndPhaseShift(90);

      Vector3D planarVelocity = new Vector3D(1.0, 0.0, 0.0);
      RobotQuadrant initialStepQuadrant = RobotQuadrant.HIND_RIGHT;
      FramePoint supportCentroidAtSoS = new FramePoint(ReferenceFrame.getWorldFrame());
      supportCentroidAtSoS.set(0, 0, 0);
      double timeAtSoS = 0.0;
      double yawAtSoS = 0.0;

      ArrayList<QuadrupedTimedStep> plannedSteps = new ArrayList<>();
      for (int i = 0; i < 4; i++)
      {
         plannedSteps.add(new QuadrupedTimedStep());
      }

      xGaitPlanner.computeInitialPlan(plannedSteps, planarVelocity, initialStepQuadrant, supportCentroidAtSoS, timeAtSoS, yawAtSoS, xGaitSettings);

      ArrayList<QuadrupedTimedStep> nominalSteps = new ArrayList<>();
      for (int i = 0; i < 4; i++)
      {
         nominalSteps.add(new QuadrupedTimedStep());
      }

      nominalSteps.get(0).setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
      nominalSteps.get(0).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(0).setGoalPosition(new Point3D(-0.25, -0.125, 0.0));
      nominalSteps.get(0).getTimeInterval().setInterval(0.0, 0.25);

      nominalSteps.get(1).setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      nominalSteps.get(1).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(1).setGoalPosition(new Point3D(0.875, -0.125, 0.0));
      nominalSteps.get(1).getTimeInterval().setInterval(0.125, 0.375);

      nominalSteps.get(2).setRobotQuadrant(RobotQuadrant.HIND_LEFT);
      nominalSteps.get(2).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(2).setGoalPosition(new Point3D(0.0, 0.125, 0.0));
      nominalSteps.get(2).getTimeInterval().setInterval(0.25, 0.5);

      nominalSteps.get(3).setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      nominalSteps.get(3).setGroundClearance(xGaitSettings.getStepGroundClearance());
      nominalSteps.get(3).setGoalPosition(new Point3D(1.125, 0.125, 0.0));
      nominalSteps.get(3).getTimeInterval().setInterval(0.375, 0.625);

      double epsilon = 0.00001;
      for (int i = 0; i < 4; i++)
      {
         assertTrue("planned step " + i  + " does not match nominal step", plannedSteps.get(i).epsilonEquals(nominalSteps.get(i), epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testOnlineForwardVelocityPlan()
   {
      QuadrupedXGaitPlanner xGaitPlanner = new QuadrupedXGaitPlanner();
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.25);
      xGaitSettings.setStepGroundClearance(0.1);
      xGaitSettings.setStepDuration(0.25);
      xGaitSettings.setEndDoubleSupportDuration(0);
      xGaitSettings.setEndPhaseShift(90);

      Vector3D planarVelocity = new Vector3D(1.0, 0.0, 0.0);
      double currentTime = 0.125;
      double currentYaw = 0.0;

      EndDependentList<QuadrupedTimedStep> priorSteps = new EndDependentList<>();
      priorSteps.set(RobotEnd.HIND, new QuadrupedTimedStep());
      priorSteps.get(RobotEnd.HIND).setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
      priorSteps.get(RobotEnd.HIND).setGroundClearance(xGaitSettings.getStepGroundClearance());
      priorSteps.get(RobotEnd.HIND).setGoalPosition(new Point3D(-0.25, -0.125, 0.0));
      priorSteps.get(RobotEnd.HIND).getTimeInterval().setInterval(0.0, 0.25);

      priorSteps.set(RobotEnd.FRONT, new QuadrupedTimedStep());
      priorSteps.get(RobotEnd.FRONT).setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      priorSteps.get(RobotEnd.FRONT).setGroundClearance(xGaitSettings.getStepGroundClearance());
      priorSteps.get(RobotEnd.FRONT).setGoalPosition(new Point3D(0.875, -0.125, 0.0));
      priorSteps.get(RobotEnd.FRONT).getTimeInterval().setInterval(0.125, 0.375);

      ArrayList<QuadrupedTimedStep> plannedSteps = new ArrayList<>();
      for (int i = 0; i < 2; i++)
      {
         plannedSteps.add(new QuadrupedTimedStep());
      }
      xGaitPlanner.computeOnlinePlan(plannedSteps, priorSteps, planarVelocity, currentTime, currentYaw, xGaitSettings);

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
      for (int i = 0; i < 2; i++)
      {
         assertTrue("planned step " + i  + " does not match nominal step", plannedSteps.get(i).epsilonEquals(nominalSteps.get(i), epsilon));
      }
   }
}