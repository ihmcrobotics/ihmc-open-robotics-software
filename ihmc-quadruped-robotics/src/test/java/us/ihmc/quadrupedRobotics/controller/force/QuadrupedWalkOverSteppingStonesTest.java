package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.*;
import org.junit.After;
import org.junit.Before;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.communication.QuadrupedNetClassList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.simulationConstructionSetTools.util.environments.SteppingStonesEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public abstract class QuadrupedWalkOverSteppingStonesTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testWalkOverSteppingStones() throws IOException, BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      SteppingStonesEnvironment environment = new SteppingStonesEnvironment();
      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);
      QuadrupedInitialOffsetAndYaw initialOffset = new QuadrupedInitialOffsetAndYaw(new Vector3D(environment.getStartPosition()), environment.getStartYaw());

      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setInitialOffset(initialOffset);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);

      quadrupedTestFactory.setTerrainObject3D(environment.getTerrainObject3D());
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      PacketCommunicator packetCommunicator = PacketCommunicator
            .createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new QuadrupedNetClassList());
      AtomicReference<QuadrupedControllerEnum> controllerState = new AtomicReference<>();
      AtomicReference<QuadrupedSteppingStateEnum> steppingState = new AtomicReference<>();
      packetCommunicator.attachListener(QuadrupedControllerStateChangeMessage.class,
                                        packet -> controllerState.set(QuadrupedControllerEnum.fromByte(packet.getEndControllerName())));

      packetCommunicator.attachListener(QuadrupedSteppingStateChangeMessage.class,
                                        packet -> steppingState.set(QuadrupedSteppingStateEnum.fromByte(packet.getEndSteppingControllerName())));
      packetCommunicator.connect();

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      List<QuadrupedTimedStepMessage> steps = getSteps(environment.getBaseBlockFrame());
      QuadrupedTimedStepListMessage message = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(steps, false);
      packetCommunicator.send(message);

      boolean isStanding = true;
      while (isStanding)
      {
         conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
         conductor.simulate();
         isStanding = steppingState.get() == QuadrupedSteppingStateEnum.STAND;
      }

      boolean isStepping = true;
      while (isStepping)
      {
         conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
         conductor.simulate();
         isStepping = steppingState.get() == QuadrupedSteppingStateEnum.STEP;
      }

      conductor.concludeTesting();
   }

   private List<QuadrupedTimedStepMessage> getSteps(ReferenceFrame baseBlockFrame)
   {
      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();

      FramePoint3D goalPosition1 = new FramePoint3D(baseBlockFrame, 0.7, 0.075, 0.0);
      FramePoint3D goalPosition2 = new FramePoint3D(baseBlockFrame, -0.25, -0.20, 0.0);
      FramePoint3D goalPosition3 = new FramePoint3D(baseBlockFrame, 0.7, -0.20, 0.0);
      FramePoint3D goalPosition4 = new FramePoint3D(baseBlockFrame, -0.2, 0.05, 0.0);

      FramePoint3D goalPosition5 = new FramePoint3D(baseBlockFrame, 0.875, 0.10, 0.0);
      FramePoint3D goalPosition6 = new FramePoint3D(baseBlockFrame, -0.05, -0.30, 0.0);
      FramePoint3D goalPosition7 = new FramePoint3D(baseBlockFrame, 0.9, -0.35, 0.0);
      FramePoint3D goalPosition8 = new FramePoint3D(baseBlockFrame, -0.05, 0.075, 0.0);

      FramePoint3D goalPosition9 = new FramePoint3D(baseBlockFrame, 0.975, 0.10, 0.0);
      FramePoint3D goalPosition10 = new FramePoint3D(baseBlockFrame, 0.05, -0.32, 0.0);
      FramePoint3D goalPosition11 = new FramePoint3D(baseBlockFrame, 1.05, -0.35, 0.0);




      goalPosition1.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition2.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition3.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition4.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition5.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition6.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition7.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition8.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition9.changeFrame(ReferenceFrame.getWorldFrame());

      QuadrupedTimedStepMessage message1 = new QuadrupedTimedStepMessage();
      message1.getTimeInterval().setStartTime(0.0);
      message1.getTimeInterval().setEndTime(0.4);
      message1.getQuadrupedStepMessage().setGroundClearance(0.1);
      message1.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message1.getQuadrupedStepMessage().getGoalPosition().set(goalPosition1);

      QuadrupedTimedStepMessage message2 = new QuadrupedTimedStepMessage();
      message2.getTimeInterval().setStartTime(0.2);
      message2.getTimeInterval().setEndTime(0.6);
      message2.getQuadrupedStepMessage().setGroundClearance(0.1);
      message2.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message2.getQuadrupedStepMessage().getGoalPosition().set(goalPosition2);

      QuadrupedTimedStepMessage message3 = new QuadrupedTimedStepMessage();
      message3.getTimeInterval().setStartTime(0.4);
      message3.getTimeInterval().setEndTime(0.8);
      message3.getQuadrupedStepMessage().setGroundClearance(0.1);
      message3.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message3.getQuadrupedStepMessage().getGoalPosition().set(goalPosition3);

      QuadrupedTimedStepMessage message4 = new QuadrupedTimedStepMessage();
      message4.getTimeInterval().setStartTime(0.6);
      message4.getTimeInterval().setEndTime(1.0);
      message4.getQuadrupedStepMessage().setGroundClearance(0.1);
      message4.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message4.getQuadrupedStepMessage().getGoalPosition().set(goalPosition4);



      QuadrupedTimedStepMessage message5 = new QuadrupedTimedStepMessage();
      message5.getTimeInterval().setStartTime(0.8);
      message5.getTimeInterval().setEndTime(1.2);
      message5.getQuadrupedStepMessage().setGroundClearance(0.1);
      message5.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message5.getQuadrupedStepMessage().getGoalPosition().set(goalPosition5);

      QuadrupedTimedStepMessage message6 = new QuadrupedTimedStepMessage();
      message6.getTimeInterval().setStartTime(1.0);
      message6.getTimeInterval().setEndTime(1.4);
      message6.getQuadrupedStepMessage().setGroundClearance(0.1);
      message6.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message6.getQuadrupedStepMessage().getGoalPosition().set(goalPosition6);

      QuadrupedTimedStepMessage message7 = new QuadrupedTimedStepMessage();
      message7.getTimeInterval().setStartTime(1.4);
      message7.getTimeInterval().setEndTime(1.8);
      message7.getQuadrupedStepMessage().setGroundClearance(0.1);
      message7.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message7.getQuadrupedStepMessage().getGoalPosition().set(goalPosition7);

      QuadrupedTimedStepMessage message8 = new QuadrupedTimedStepMessage();
      message8.getTimeInterval().setStartTime(1.8);
      message8.getTimeInterval().setEndTime(2.2);
      message8.getQuadrupedStepMessage().setGroundClearance(0.1);
      message8.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message8.getQuadrupedStepMessage().getGoalPosition().set(goalPosition8);



      QuadrupedTimedStepMessage message9 = new QuadrupedTimedStepMessage();
      message9.getTimeInterval().setStartTime(2.0);
      message9.getTimeInterval().setEndTime(2.4);
      message9.getQuadrupedStepMessage().setGroundClearance(0.1);
      message9.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message9.getQuadrupedStepMessage().getGoalPosition().set(goalPosition9);

      QuadrupedTimedStepMessage message10 = new QuadrupedTimedStepMessage();
      message10.getTimeInterval().setStartTime(2.2);
      message10.getTimeInterval().setEndTime(2.6);
      message10.getQuadrupedStepMessage().setGroundClearance(0.1);
      message10.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message10.getQuadrupedStepMessage().getGoalPosition().set(goalPosition10);

      QuadrupedTimedStepMessage message11 = new QuadrupedTimedStepMessage();
      message11.getTimeInterval().setStartTime(2.4);
      message11.getTimeInterval().setEndTime(2.8);
      message11.getQuadrupedStepMessage().setGroundClearance(0.1);
      message11.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message11.getQuadrupedStepMessage().getGoalPosition().set(goalPosition11);

      steps.add(message1);
      steps.add(message2);
      steps.add(message3);
      steps.add(message4);

      steps.add(message5);
      steps.add(message6);
      steps.add(message7);
      steps.add(message8);

      steps.add(message9);
      steps.add(message10);
      steps.add(message11);

      return steps;
   }
}
