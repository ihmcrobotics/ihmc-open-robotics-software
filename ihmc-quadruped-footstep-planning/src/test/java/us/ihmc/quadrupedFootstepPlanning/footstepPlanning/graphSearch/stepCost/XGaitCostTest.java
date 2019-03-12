package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import org.junit.jupiter.api.Test;
import org.omg.PortableServer.IMPLICIT_ACTIVATION_POLICY_ID;
import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import static us.ihmc.robotics.Assert.assertEquals;
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
      xGaitSettings.setEndPhaseShift(0);
      xGaitSettings.setStepDuration(stepDuration);
      xGaitSettings.setEndDoubleSupportDuration(doubleSupportDuration);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings);

      String errorMessage = "";
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.FRONT_LEFT, xGaitCost);
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.FRONT_RIGHT, xGaitCost);

      errorMessage += testTimeDelta(0.0, RobotQuadrant.HIND_RIGHT, xGaitCost);
      errorMessage += testTimeDelta(0.0, RobotQuadrant.HIND_LEFT, xGaitCost);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }

   @Test
   public void testComputeTimeDeltaBetweenStepsCrawl()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(90);
      xGaitSettings.setStepDuration(stepDuration);
      xGaitSettings.setEndDoubleSupportDuration(doubleSupportDuration);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings);

      String errorMessage = "";
      errorMessage += testTimeDelta(0.5 * stepDuration, RobotQuadrant.FRONT_LEFT, xGaitCost);
      errorMessage += testTimeDelta(0.5 * stepDuration, RobotQuadrant.FRONT_RIGHT, xGaitCost);

      errorMessage += testTimeDelta(0.5 * stepDuration + doubleSupportDuration, RobotQuadrant.HIND_RIGHT, xGaitCost);
      errorMessage += testTimeDelta(0.5 * stepDuration + doubleSupportDuration, RobotQuadrant.HIND_LEFT, xGaitCost);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }

   @Test
   public void testComputeTimeDeltaBetweenStepsTrot()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(90);
      xGaitSettings.setStepDuration(stepDuration);
      xGaitSettings.setEndDoubleSupportDuration(doubleSupportDuration);

      FootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      XGaitCost xGaitCost = new XGaitCost(footstepPlannerParameters, xGaitSettings);

      String errorMessage = "";
      errorMessage += testTimeDelta(0.0, RobotQuadrant.FRONT_LEFT, xGaitCost);
      errorMessage += testTimeDelta(0.0, RobotQuadrant.FRONT_RIGHT, xGaitCost);

      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.HIND_RIGHT, xGaitCost);
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.HIND_LEFT, xGaitCost);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }

   private String testTimeDelta( double expectedDuration, RobotQuadrant robotQuadrant, XGaitCost xGaitCost)
   {
      String message = "";
      double actual = xGaitCost.computeTimeDeltaBetweenSteps(robotQuadrant);
      if (!MathTools.epsilonEquals(expectedDuration, actual, epsilon))
         message += "\n" + robotQuadrant + " expected duration " + expectedDuration + ", got " + actual;

      return message;
   }
}
