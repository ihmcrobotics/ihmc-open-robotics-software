package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Tag("quadruped-xgait")
public abstract class QuadrupedXGaitPushRecoveryTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
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
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUsePushRobotController(true);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
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

   public abstract double getWalkingSpeed();
   public abstract double getStepDuration();

   @Test
   public void testWalkingForwardFastWithPush()
   {
      testWalkingWithPush(90.0, getWalkingSpeed());
   }

   @Test
   public void testScriptedWalkingForwardFastWithPush()
   {
      testScriptedWalkingWithPush(90.0, getWalkingSpeed(), getStepDuration());
   }

   private void testWalkingWithPush(double endPhaseShift, double walkingSpeed)
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      stepTeleopManager.requestXGait();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, -1.0, 0.0), 45.0, 0.3);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 4.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), 35.0, 0.15);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 10.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 7.0));
      conductor.simulate();
   }

   private void testScriptedWalkingWithPush(double endPhaseShift, double walkingSpeed, double stepDuration)
   {

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      double stepLength = walkingSpeed * (2 * stepDuration);
      List<QuadrupedTimedStepMessage> steps = getSteps(endPhaseShift, stepLength, 1.0, 0.0, stepDuration, 10);
      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      stepTeleopManager.publishTimedStepListToController(message);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, -1.0, 0.0), 50.0, 0.3);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 4.0));
      conductor.simulate();

      /*
      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), 50.0, 0.3);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 7.0));
      conductor.simulate();
      */
   }

   public List<QuadrupedTimedStepMessage> getSteps(double endPhaseShift, double stepLength, double nominalLength, double startTime, double duration, int numberOfSteps)
   {
      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();

      double width = 0.2;
      double hindAddedFraction = 0.15;
      Point3D frontLeft = new Point3D(0.5 * nominalLength + 0.5 * stepLength, 0.5 * width, 0.0);
      Point3D hindRight = new Point3D(-0.5 * nominalLength + (0.5 + hindAddedFraction) * stepLength, -0.5 * width, 0.0);
      Point3D frontRight = new Point3D(0.5 * nominalLength + stepLength, -0.5 * width, 0.0);
      Point3D hindLeft = new Point3D(-0.5 * nominalLength + (1.0 + hindAddedFraction) * stepLength, 0.5 * width, 0.0);

      TimeInterval frontLeftInterval = new TimeInterval(startTime, startTime + duration);
      TimeInterval hindRightInterval = new TimeInterval(startTime + endPhaseShift / 180.0 * duration, startTime + 1.5 * duration);
      TimeInterval frontRightInterval = new TimeInterval(frontLeftInterval);
      TimeInterval hindLeftInterval = new TimeInterval(hindRightInterval);
      frontRightInterval.shiftInterval(duration);
      hindLeftInterval.shiftInterval(duration);

      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, frontLeft, 0.1, frontLeftInterval));
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, hindRight, 0.1, hindRightInterval));
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, frontRight, 0.1, frontRightInterval));
         steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, hindLeft, 0.1, hindLeftInterval));


         frontLeft.addX(stepLength);
         hindRight.addX(stepLength);
         frontRight.addX(stepLength);
         hindLeft.addX(stepLength);

         frontLeftInterval.shiftInterval(2.0 * duration);
         hindRightInterval.shiftInterval(2.0 * duration);
         frontRightInterval.shiftInterval(2.0 * duration);
         hindLeftInterval.shiftInterval(2.0 * duration);
      }


      frontRight.set(frontLeft);
      frontRight.setY(-0.5 * width);

      hindLeft.set(frontLeft);
      hindLeft.subX(nominalLength);
      hindRight.set(frontRight);
      hindRight.subX(nominalLength);

      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, frontLeft, 0.1, frontLeftInterval));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, hindRight, 0.1, hindRightInterval));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, frontRight, 0.1, frontRightInterval));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, hindLeft, 0.1, hindLeftInterval));

      /*
      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(1.45, 0.1, -0.000), 0.1, new TimeInterval(1.1, 1.4)));
      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.5, -0.1, -0.000), 0.1, new TimeInterval(1.25, 1.55)));
      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.65, -0.1, -0.000), 0.1, new TimeInterval(1.4, 1.7)));
      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(0.65, 0.1, -0.000), 0.1, new TimeInterval(1.55, 1.85)));

      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(2.05, 0.1, -0.000), 0.1, new TimeInterval(1.7, 2.0)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(1.15, -0.1, -0.000), 0.1, new TimeInterval(1.85, 2.15)));
      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(2.25, -0.1, -0.000), 0.1, new TimeInterval(2.0, 2.3)));
      steps.add(
            QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(1.25, 0.1, -0.000), 0.1, new TimeInterval(2.15, 2.45)));
            */

      return steps;
   }
}
