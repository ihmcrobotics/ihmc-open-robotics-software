package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedScriptedFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   protected RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @BeforeEach
   public void setup() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testScriptedFlatGroundWalking() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestWalkingState();

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      List<QuadrupedTimedStepMessage> steps = getSteps();
      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      stepTeleopManager.publishTimedStepListToController(message);

      // check robot is still upright and walked forward
      Point3D expectedFinalPlanarPosition = getFinalPlanarPosition();
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), expectedFinalPlanarPosition.getX(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), expectedFinalPlanarPosition.getY(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), expectedFinalPlanarPosition.getZ(), 0.1));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, message.getQuadrupedStepList().getLast().getTimeInterval().getEndTime() + 2.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));

      conductor.simulate();

      conductor.concludeTesting();
   }

   public void testScriptedFlatGroundWalkingWithHeightOffset(double stepHeightOffset) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestWalkingState();

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      Point3D initialPosition = new Point3D(quadrupedTestFactory.getFullRobotModel().getBody().getParentJoint().getFrameAfterJoint().getTransformToRoot().getTranslationVector());

      List<QuadrupedTimedStepMessage> steps = getSteps();
      steps.forEach(step -> step.getQuadrupedStepMessage().getGoalPosition().addZ(stepHeightOffset));

      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      message.setOffsetStepsHeightWithExecutionError(true);
      stepTeleopManager.publishTimedStepListToController(message);

      // check robot is still upright and walked forward
      Point3D expectedFinalPlanarPosition = getFinalPlanarPosition();
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), expectedFinalPlanarPosition.getX(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), expectedFinalPlanarPosition.getY(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), expectedFinalPlanarPosition.getZ(), 0.1));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, message.getQuadrupedStepList().getLast().getTimeInterval().getEndTime() + 2.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));

      conductor.simulate();

      // Re-execute the same footsteps but shifted to the new robot location
      Point3D currentPosition = new Point3D(quadrupedTestFactory.getFullRobotModel().getBody().getParentJoint().getFrameAfterJoint().getTransformToRoot().getTranslationVector());
      Vector3D offset = new Vector3D();
      offset.sub(currentPosition, initialPosition);
      offset.setZ(0.0);
      steps.forEach(step -> step.getQuadrupedStepMessage().getGoalPosition().add(offset));
      message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      message.setOffsetStepsHeightWithExecutionError(true);
      stepTeleopManager.publishTimedStepListToController(message);

      // check robot is still upright and walked forward
      expectedFinalPlanarPosition.add(offset);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), expectedFinalPlanarPosition.getX(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), expectedFinalPlanarPosition.getY(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), expectedFinalPlanarPosition.getZ(), 0.1));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, message.getQuadrupedStepList().getLast().getTimeInterval().getEndTime() + 2.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));

      conductor.simulate();

      conductor.concludeTesting();
   }

   @Test
   public void testScriptedTroublingSteps()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestWalkingState();

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      double groundClearance = 0.05;

      List<QuadrupedTimedStepMessage> steps = new ArrayList<>();
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.55, 0.1, 0.0), groundClearance, 0.1, 0.43));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.3, 0.05, 0.0), groundClearance, 0.53, 0.86));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.85, -0.05, 0.0), groundClearance, 0.96, 1.29));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.15, 0.3, 0.0), groundClearance, 1.39, 1.72));

      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(1.05, 0.15, 0.0), groundClearance, 1.82, 2.15));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.1, 0.15, 0.0), groundClearance, 2.25, 2.58));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.3, 0.1, 0.0), groundClearance, 2.68, 3.01));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(0.3, 0.4, 0.0), groundClearance, 3.11, 3.44));

      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(1.5, 0.25, 0.0), groundClearance, 3.54, 3.87));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.5, 0.25, 0.0), groundClearance, 3.97, 4.3));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.7, 0.2, 0.0), groundClearance, 4.4, 4.73));
      steps.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(0.7, 0.5, 0.0), groundClearance, 4.83, 5.16));

      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      stepTeleopManager.publishTimedStepListToController(message);

      // check robot is still upright and walked forward
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 1.05, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), 0.3, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -0.15, 0.1));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, message.getQuadrupedStepList().getLast().getTimeInterval().getEndTime() + 2.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));

      conductor.simulate();

      conductor.concludeTesting();
   }

   @Test
   public void testPauseWalking()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestWalkingState();

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      List<QuadrupedTimedStepMessage> steps = getSteps();
      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      stepTeleopManager.publishTimedStepListToController(message);

      double halfwayTime = steps.get(steps.size() / 2).getTimeInterval().getStartTime();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, halfwayTime));
      conductor.simulate();

      stepTeleopManager.requestPauseWalking(true);

      // give the robot time to finish step
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
      Assertions.assertTrue(variables.getSteppingState().getEnumValue() == QuadrupedSteppingStateEnum.STAND);

      double robotBodyX = variables.getRobotBodyX().getDoubleValue();
      double robotBodyY = variables.getRobotBodyY().getDoubleValue();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 3.0));
      conductor.simulate();

      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(robotBodyX, variables.getRobotBodyX().getDoubleValue(), 1e-2));
      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(robotBodyY, variables.getRobotBodyY().getDoubleValue(), 1e-2));

      stepTeleopManager.requestPauseWalking(false);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.5));
      conductor.simulate();
      Assertions.assertTrue(variables.getSteppingState().getEnumValue() == QuadrupedSteppingStateEnum.STEP);

      // check that steps are preserved when walking is resumed
      Point3D expectedFinalPlanarPosition = getFinalPlanarPosition();
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), expectedFinalPlanarPosition.getX(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), expectedFinalPlanarPosition.getY(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), expectedFinalPlanarPosition.getZ(), 0.1));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, message.getQuadrupedStepList().getLast().getTimeInterval().getEndTime() + 2.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.simulate();

      conductor.concludeTesting();
   }

   @Test
   public void testAbortWalking()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestWalkingState();

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      List<QuadrupedTimedStepMessage> steps = getSteps();
      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      stepTeleopManager.publishTimedStepListToController(message);

      double halfwayTime = steps.get(steps.size() / 2).getTimeInterval().getStartTime();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, halfwayTime));
      conductor.simulate();

      stepTeleopManager.requestAbortWalking();

      // give the robot time to finish step
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
      Assertions.assertTrue(variables.getSteppingState().getEnumValue() == QuadrupedSteppingStateEnum.STAND);

      double robotBodyX = variables.getRobotBodyX().getDoubleValue();
      double robotBodyY = variables.getRobotBodyY().getDoubleValue();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 3.0));
      conductor.simulate();

      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(robotBodyX, variables.getRobotBodyX().getDoubleValue(), 1e-2));
      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(robotBodyY, variables.getRobotBodyY().getDoubleValue(), 1e-2));

      // send the second half of the step list, check that the robot resumes walking when new steps are received
      int initialSize = steps.size();
      while (steps.size() > initialSize / 2)
      {
         steps.remove(0);
      }
      message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      stepTeleopManager.publishTimedStepListToController(message);

      Point3D expectedFinalPlanarPosition = getFinalPlanarPosition();
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), expectedFinalPlanarPosition.getX(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), expectedFinalPlanarPosition.getY(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), expectedFinalPlanarPosition.getZ(), 0.1));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, message.getQuadrupedStepList().getLast().getTimeInterval().getEndTime() + 2.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.simulate();

      conductor.concludeTesting();
   }

   /**
    * Steps to execute, not expressed in absolute time
    */
   public abstract List<QuadrupedTimedStepMessage> getSteps();

   /**
    * Expected final planar position, given as x, y, yaw
    */
   public abstract Point3D getFinalPlanarPosition();
}
