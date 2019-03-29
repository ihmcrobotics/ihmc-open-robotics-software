package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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

   /**
    * Steps to execute, not expressed in absolute time
    */
   public abstract List<QuadrupedTimedStepMessage> getSteps();

   /**
    * Expected final planar position, given as x, y, yaw
    */
   public abstract Point3D getFinalPlanarPosition();
}
