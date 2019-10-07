package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.QuadrupedStepMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class QuadrupedContinuousPlanningControllerTest
{
   @Test
   public void testSortingAmbleSteps()
   {
      QuadrupedTimedStep step0 = new QuadrupedTimedStep();
      QuadrupedTimedStep step1 = new QuadrupedTimedStep();
      QuadrupedTimedStep step2 = new QuadrupedTimedStep();
      QuadrupedTimedStep step3 = new QuadrupedTimedStep();

      List<QuadrupedTimedStep> steps = new ArrayList<>();
      steps.add(step0);
      steps.add(step1);
      steps.add(step2);
      steps.add(step3);

      step0.getTimeInterval().setStartTime(0.0);
      step0.getTimeInterval().setEndTime(1.0);

      step1.getTimeInterval().setStartTime(0.25);
      step1.getTimeInterval().setEndTime(1.25);

      step2.getTimeInterval().setStartTime(0.5);
      step2.getTimeInterval().setEndTime(1.5);

      step3.getTimeInterval().setStartTime(0.75);
      step3.getTimeInterval().setEndTime(1.75);

      steps.sort(QuadrupedContinuousPlanningController.startTimeComparator);

      assertEquals(step0, steps.get(0));
      assertEquals(step1, steps.get(1));
      assertEquals(step2, steps.get(2));
      assertEquals(step3, steps.get(3));

      steps.clear();
      steps.add(step3);
      steps.add(step1);
      steps.add(step0);
      steps.add(step2);
      steps.sort(QuadrupedContinuousPlanningController.startTimeComparator);

      assertEquals(step0, steps.get(0));
      assertEquals(step1, steps.get(1));
      assertEquals(step2, steps.get(2));
      assertEquals(step3, steps.get(3));
   }

   @Test
   public void testSortingTrotSteps()
   {
      QuadrupedTimedStep step0 = new QuadrupedTimedStep();
      QuadrupedTimedStep step1 = new QuadrupedTimedStep();
      QuadrupedTimedStep step2 = new QuadrupedTimedStep();
      QuadrupedTimedStep step3 = new QuadrupedTimedStep();

      List<QuadrupedTimedStep> steps = new ArrayList<>();
      steps.add(step0);
      steps.add(step1);
      steps.add(step2);
      steps.add(step3);

      step0.getTimeInterval().setStartTime(0.0);
      step0.getTimeInterval().setEndTime(1.0);
      step0.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);

      step1.getTimeInterval().setStartTime(0.0);
      step1.getTimeInterval().setEndTime(1.0);
      step1.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);

      step2.getTimeInterval().setStartTime(0.25);
      step2.getTimeInterval().setEndTime(1.25);
      step2.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);

      step3.getTimeInterval().setStartTime(0.25);
      step3.getTimeInterval().setEndTime(1.25);
      step3.setRobotQuadrant(RobotQuadrant.HIND_LEFT);

      steps.sort(QuadrupedContinuousPlanningController.startTimeComparator);

      assertEquals(step0, steps.get(0));
      assertEquals(step1, steps.get(1));
      assertEquals(step2, steps.get(2));
      assertEquals(step3, steps.get(3));

      steps.clear();
      steps.add(step3);
      steps.add(step1);
      steps.add(step0);
      steps.add(step2);
      steps.sort(QuadrupedContinuousPlanningController.startTimeComparator);

      assertEquals(step0, steps.get(0));
      assertEquals(step1, steps.get(1));
      assertEquals(step2, steps.get(2));
      assertEquals(step3, steps.get(3));
   }

   @Test
   public void testGetLastStepInQuadrantFromList()
   {
      QuadrupedTimedStep step0 = new QuadrupedTimedStep();
      QuadrupedTimedStep step1 = new QuadrupedTimedStep();
      QuadrupedTimedStep step2 = new QuadrupedTimedStep();
      QuadrupedTimedStep step3 = new QuadrupedTimedStep();
      QuadrupedTimedStep step4 = new QuadrupedTimedStep();
      QuadrupedTimedStep step5 = new QuadrupedTimedStep();

      List<QuadrupedTimedStep> steps = new ArrayList<>();
      steps.add(step0);
      steps.add(step1);
      steps.add(step2);
      steps.add(step3);

      step0.getTimeInterval().setStartTime(0.0);
      step0.getTimeInterval().setEndTime(1.0);
      step0.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);

      step1.getTimeInterval().setStartTime(0.0);
      step1.getTimeInterval().setEndTime(1.0);
      step1.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);

      step2.getTimeInterval().setStartTime(0.25);
      step2.getTimeInterval().setEndTime(1.25);
      step2.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);

      step3.getTimeInterval().setStartTime(0.25);
      step3.getTimeInterval().setEndTime(1.25);
      step3.setRobotQuadrant(RobotQuadrant.HIND_LEFT);

      step4.getTimeInterval().setStartTime(0.5);
      step4.getTimeInterval().setEndTime(1.5);
      step4.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);

      step5.getTimeInterval().setStartTime(0.5);
      step5.getTimeInterval().setEndTime(1.5);
      step5.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);

      steps.sort(QuadrupedContinuousPlanningController.startTimeComparator);

      assertEquals(step0, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.FRONT_LEFT, steps));
      assertEquals(step1, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.HIND_RIGHT, steps));
      assertEquals(step2, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.FRONT_RIGHT, steps));
      assertEquals(step3, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.HIND_LEFT, steps));

      steps.add(step4);
      steps.add(step5);

      assertEquals(step4, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.FRONT_LEFT, steps));
      assertEquals(step5, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.HIND_RIGHT, steps));
      assertEquals(step2, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.FRONT_RIGHT, steps));
      assertEquals(step3, QuadrupedContinuousPlanningController.getLastStepInQuadrantFromList(RobotQuadrant.HIND_LEFT, steps));
   }
}
