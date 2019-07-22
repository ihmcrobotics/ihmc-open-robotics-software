package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.stepStream;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import static us.ihmc.quadrupedPlanning.QuadrupedSpeed.MEDIUM;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class QuadrupedXGaitToolsTest
{
   private static final double epsilon = 1e-8;
   private static final double stepDuration = 0.5;
   private static final double doubleSupportDuration = 0.2;

   @Test
   public void testManualTrot()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);

      assertEquals(0.0, QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(RobotQuadrant.FRONT_LEFT, xGaitSettings));
      assertEquals(0.0, QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(RobotQuadrant.FRONT_RIGHT, xGaitSettings));

      assertEquals(stepDuration + doubleSupportDuration, QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(RobotQuadrant.HIND_RIGHT, xGaitSettings));
      assertEquals(stepDuration + doubleSupportDuration, QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(RobotQuadrant.HIND_LEFT, xGaitSettings));
   }


   @Test
   public void testComputeTimeDeltaBetweenStepsPace()
   {
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setEndPhaseShift(QuadrupedGait.PACE.getEndPhaseShift());
      xGaitSettings.setQuadrupedSpeed(MEDIUM);
      xGaitSettings.getPaceMediumTimings().setStepDuration(stepDuration);
      xGaitSettings.getPaceMediumTimings().setEndDoubleSupportDuration(doubleSupportDuration);

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

      String errorMessage = "";
      errorMessage += testTimeDelta(0.0, RobotQuadrant.FRONT_LEFT, xGaitSettings);
      errorMessage += testTimeDelta(0.0, RobotQuadrant.FRONT_RIGHT, xGaitSettings);

      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.HIND_RIGHT, xGaitSettings);
      errorMessage += testTimeDelta(stepDuration + doubleSupportDuration, RobotQuadrant.HIND_LEFT, xGaitSettings);

      assertTrue(errorMessage, errorMessage.isEmpty());
   }


   private String testTimeDelta(double expectedDuration, RobotQuadrant robotQuadrant, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      String message = "";
      double actual = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(robotQuadrant, xGaitSettings);
      if (!MathTools.epsilonEquals(expectedDuration, actual, epsilon))
         message += "\n" + robotQuadrant + " expected duration " + expectedDuration + ", got " + actual;

      return message;
   }
}
