package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.io.IOException;

@Tag("quadruped-xgait")
public abstract class QuadrupedDelayedLiftOffTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private PushRobotTestConductor pusher;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUsePushRobotController(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedTestYoVariables(conductor.getScs());
         pusher = new PushRobotTestConductor(conductor.getScs(), "body");
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();

      conductor = null;
      variables = null;
      pusher = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testWalkingForwardWithPush()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      YoBoolean swingSpeedUpEnabled = (YoBoolean) conductor.getScs().getRootRegistry().getVariable("isSwingSpeedUpEnabled");
      swingSpeedUpEnabled.set(false);

      QuadrupedTimedStep step1 = new QuadrupedTimedStep();
      QuadrupedTimedStep step2 = new QuadrupedTimedStep();
      QuadrupedTimedStep step3 = new QuadrupedTimedStep();
      QuadrupedTimedStep step4 = new QuadrupedTimedStep();

      double stanceLength = stepTeleopManager.getXGaitSettings().getStanceLength();
      double stanceWidth = stepTeleopManager.getXGaitSettings().getStanceWidth();

      step1.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      step1.getTimeInterval().setInterval(0.5, 1.25);
      step1.setGoalPosition(new Point3D(stanceLength / 2.0 + 0.1, stanceWidth / 2.0, 0.0));

      step2.setRobotQuadrant(RobotQuadrant.HIND_LEFT);
      step2.getTimeInterval().setInterval(1.1, 1.85);
      step2.setGoalPosition(new Point3D(-stanceLength / 2.0 + 0.2, stanceWidth / 2.0, 0.0));

      step3.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      step3.getTimeInterval().setInterval(1.7, 2.45);
      step3.setGoalPosition(new Point3D(stanceLength / 2.0 + 0.3, -stanceWidth / 2.0, 0.0));

      step4.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
      step4.getTimeInterval().setInterval(2.3, 3.05);
      step4.setGoalPosition(new Point3D(-stanceLength / 2.0 + 0.4, -stanceWidth / 2.0, 0.0));

      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();
      stepListMessage.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(step1));
      stepListMessage.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(step2));
      stepListMessage.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(step3));
      stepListMessage.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(step4));

      stepListMessage.setIsExpressedInAbsoluteTime(false);
      stepListMessage.setAreStepsAdjustable(false);

      stepTeleopManager.publishTimedStepListToController(stepListMessage);

      YoVariableTestGoal timeGoal = YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), 4.8);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(timeGoal);
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), 100.0, 1.5);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 5.0));
      conductor.simulate();
   }


}
