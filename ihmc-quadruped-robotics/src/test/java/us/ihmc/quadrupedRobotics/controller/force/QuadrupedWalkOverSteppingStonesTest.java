package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.*;
import org.junit.After;
import org.junit.Before;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.communication.QuadrupedNetClassList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
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

      ReferenceFrameTools.clearWorldFrameTree();
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
                                        packet -> controllerState.set(QuadrupedControllerEnum.fromByte(packet.getEndQuadrupedControllerEnum())));

      packetCommunicator.attachListener(QuadrupedSteppingStateChangeMessage.class,
                                        packet -> steppingState.set(QuadrupedSteppingStateEnum.fromByte(packet.getEndQuadrupedSteppingStateEnum())));
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

      FramePoint3D goalPosition1 = new FramePoint3D(baseBlockFrame, 0.7, 0.08, 0.0);
      FramePoint3D goalPosition2 = new FramePoint3D(baseBlockFrame, -0.25, -0.25, 0.0);
      FramePoint3D goalPosition3 = new FramePoint3D(baseBlockFrame, 0.7, -0.3, 0.0);
      FramePoint3D goalPosition4 = new FramePoint3D(baseBlockFrame, -0.3, 0.08, 0.0);

      FramePoint3D goalPosition5 = new FramePoint3D(baseBlockFrame, 0.95, 0.08, 0.0);
      FramePoint3D goalPosition6 = new FramePoint3D(baseBlockFrame, -0.0, -0.35, 0.0);
      FramePoint3D goalPosition7 = new FramePoint3D(baseBlockFrame, 0.9, -0.35, 0.0);
      FramePoint3D goalPosition8 = new FramePoint3D(baseBlockFrame, 0.1, 0.08, 0.0);

      FramePoint3D goalPosition9 = new FramePoint3D(baseBlockFrame, 1.35, 0.08, 0.0);
      FramePoint3D goalPosition10 = new FramePoint3D(baseBlockFrame, 0.4, -0.35, 0.0);
      FramePoint3D goalPosition11 = new FramePoint3D(baseBlockFrame, 1.25, -0.35, 0.0);
      FramePoint3D goalPosition12 = new FramePoint3D(baseBlockFrame, 0.55, 0.08, 0.0);

      FramePoint3D goalPosition13 = new FramePoint3D(baseBlockFrame, 1.7, 0.15, 0.0);
      FramePoint3D goalPosition14 = new FramePoint3D(baseBlockFrame, 0.85, -0.35, 0.0);
      FramePoint3D goalPosition15 = new FramePoint3D(baseBlockFrame, 1.7, -0.2, 0.0);
      FramePoint3D goalPosition16 = new FramePoint3D(baseBlockFrame, 1.0, 0.08, 0.0);


      FramePoint3D goalPosition17 = new FramePoint3D(baseBlockFrame, 1.95, 0.15, 0.0);
      FramePoint3D goalPosition18 = new FramePoint3D(baseBlockFrame, 1.25, -0.35, 0.0);
      FramePoint3D goalPosition19 = new FramePoint3D(baseBlockFrame, 2.0, -0.2, 0.0);
      FramePoint3D goalPosition20 = new FramePoint3D(baseBlockFrame, 1.4, 0.08, 0.0);



      FramePoint3D goalPosition21 = new FramePoint3D(baseBlockFrame, 2.45, 0.15, 0.0);
      FramePoint3D goalPosition22 = new FramePoint3D(baseBlockFrame, 1.65, -0.2, 0.0);
      FramePoint3D goalPosition23 = new FramePoint3D(baseBlockFrame, 2.55, -0.2, 0.0);
      FramePoint3D goalPosition24 = new FramePoint3D(baseBlockFrame, 1.75, 0.15, 0.0);




      FramePoint3D goalPosition25 = new FramePoint3D(baseBlockFrame, 2.95, 0.10, 0.0);
      FramePoint3D goalPosition26 = new FramePoint3D(baseBlockFrame, 2.1, -0.2, 0.0);
      FramePoint3D goalPosition27 = new FramePoint3D(baseBlockFrame, 3.1, -0.15, 0.0);
      FramePoint3D goalPosition28 = new FramePoint3D(baseBlockFrame, 2.1, 0.15, 0.0);

      FramePoint3D goalPosition29 = new FramePoint3D(baseBlockFrame, 3.55, 0.10, 0.0);
      FramePoint3D goalPosition30 = new FramePoint3D(baseBlockFrame, 2.45, -0.15, 0.0);
      FramePoint3D goalPosition31 = new FramePoint3D(baseBlockFrame, 3.55, -0.15, 0.0);
      FramePoint3D goalPosition32 = new FramePoint3D(baseBlockFrame, 2.45, 0.10, 0.0);



















      goalPosition1.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition2.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition3.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition4.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition5.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition6.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition7.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition8.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition9.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition10.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition11.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition12.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition13.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition14.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition15.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition16.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition17.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition18.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition19.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition20.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition21.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition22.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition23.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition24.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition25.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition26.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition27.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition28.changeFrame(ReferenceFrame.getWorldFrame());

      goalPosition29.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition30.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition31.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition32.changeFrame(ReferenceFrame.getWorldFrame());






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
      message7.getTimeInterval().setStartTime(1.2);
      message7.getTimeInterval().setEndTime(1.6);
      message7.getQuadrupedStepMessage().setGroundClearance(0.1);
      message7.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message7.getQuadrupedStepMessage().getGoalPosition().set(goalPosition7);

      QuadrupedTimedStepMessage message8 = new QuadrupedTimedStepMessage();
      message8.getTimeInterval().setStartTime(1.4);
      message8.getTimeInterval().setEndTime(1.8);
      message8.getQuadrupedStepMessage().setGroundClearance(0.1);
      message8.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message8.getQuadrupedStepMessage().getGoalPosition().set(goalPosition8);



      QuadrupedTimedStepMessage message9 = new QuadrupedTimedStepMessage();
      message9.getTimeInterval().setStartTime(1.6);
      message9.getTimeInterval().setEndTime(2.0);
      message9.getQuadrupedStepMessage().setGroundClearance(0.1);
      message9.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message9.getQuadrupedStepMessage().getGoalPosition().set(goalPosition9);

      QuadrupedTimedStepMessage message10 = new QuadrupedTimedStepMessage();
      message10.getTimeInterval().setStartTime(1.8);
      message10.getTimeInterval().setEndTime(2.2);
      message10.getQuadrupedStepMessage().setGroundClearance(0.1);
      message10.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message10.getQuadrupedStepMessage().getGoalPosition().set(goalPosition10);

      QuadrupedTimedStepMessage message11 = new QuadrupedTimedStepMessage();
      message11.getTimeInterval().setStartTime(2.0);
      message11.getTimeInterval().setEndTime(2.4);
      message11.getQuadrupedStepMessage().setGroundClearance(0.1);
      message11.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message11.getQuadrupedStepMessage().getGoalPosition().set(goalPosition11);

      QuadrupedTimedStepMessage message12 = new QuadrupedTimedStepMessage();
      message12.getTimeInterval().setStartTime(2.2);
      message12.getTimeInterval().setEndTime(2.6);
      message12.getQuadrupedStepMessage().setGroundClearance(0.1);
      message12.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message12.getQuadrupedStepMessage().getGoalPosition().set(goalPosition12);





      QuadrupedTimedStepMessage message13 = new QuadrupedTimedStepMessage();
      message13.getTimeInterval().setStartTime(2.4);
      message13.getTimeInterval().setEndTime(2.8);
      message13.getQuadrupedStepMessage().setGroundClearance(0.1);
      message13.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message13.getQuadrupedStepMessage().getGoalPosition().set(goalPosition13);

      QuadrupedTimedStepMessage message14 = new QuadrupedTimedStepMessage();
      message14.getTimeInterval().setStartTime(2.6);
      message14.getTimeInterval().setEndTime(3.0);
      message14.getQuadrupedStepMessage().setGroundClearance(0.1);
      message14.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message14.getQuadrupedStepMessage().getGoalPosition().set(goalPosition14);

      QuadrupedTimedStepMessage message15 = new QuadrupedTimedStepMessage();
      message15.getTimeInterval().setStartTime(2.8);
      message15.getTimeInterval().setEndTime(3.2);
      message15.getQuadrupedStepMessage().setGroundClearance(0.1);
      message15.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message15.getQuadrupedStepMessage().getGoalPosition().set(goalPosition15);

      QuadrupedTimedStepMessage message16 = new QuadrupedTimedStepMessage();
      message16.getTimeInterval().setStartTime(3.0);
      message16.getTimeInterval().setEndTime(3.4);
      message16.getQuadrupedStepMessage().setGroundClearance(0.1);
      message16.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message16.getQuadrupedStepMessage().getGoalPosition().set(goalPosition16);



      QuadrupedTimedStepMessage message17 = new QuadrupedTimedStepMessage();
      message17.getTimeInterval().setStartTime(3.2);
      message17.getTimeInterval().setEndTime(3.6);
      message17.getQuadrupedStepMessage().setGroundClearance(0.1);
      message17.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message17.getQuadrupedStepMessage().getGoalPosition().set(goalPosition17);

      QuadrupedTimedStepMessage message18 = new QuadrupedTimedStepMessage();
      message18.getTimeInterval().setStartTime(3.4);
      message18.getTimeInterval().setEndTime(3.8);
      message18.getQuadrupedStepMessage().setGroundClearance(0.1);
      message18.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message18.getQuadrupedStepMessage().getGoalPosition().set(goalPosition18);

      QuadrupedTimedStepMessage message19 = new QuadrupedTimedStepMessage();
      message19.getTimeInterval().setStartTime(3.6);
      message19.getTimeInterval().setEndTime(4.0);
      message19.getQuadrupedStepMessage().setGroundClearance(0.1);
      message19.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message19.getQuadrupedStepMessage().getGoalPosition().set(goalPosition19);

      QuadrupedTimedStepMessage message20 = new QuadrupedTimedStepMessage();
      message20.getTimeInterval().setStartTime(3.8);
      message20.getTimeInterval().setEndTime(4.2);
      message20.getQuadrupedStepMessage().setGroundClearance(0.1);
      message20.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message20.getQuadrupedStepMessage().getGoalPosition().set(goalPosition20);



      QuadrupedTimedStepMessage message21 = new QuadrupedTimedStepMessage();
      message21.getTimeInterval().setStartTime(4.0);
      message21.getTimeInterval().setEndTime(4.4);
      message21.getQuadrupedStepMessage().setGroundClearance(0.1);
      message21.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message21.getQuadrupedStepMessage().getGoalPosition().set(goalPosition21);

      QuadrupedTimedStepMessage message22 = new QuadrupedTimedStepMessage();
      message22.getTimeInterval().setStartTime(4.2);
      message22.getTimeInterval().setEndTime(4.6);
      message22.getQuadrupedStepMessage().setGroundClearance(0.1);
      message22.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message22.getQuadrupedStepMessage().getGoalPosition().set(goalPosition22);

      QuadrupedTimedStepMessage message23 = new QuadrupedTimedStepMessage();
      message23.getTimeInterval().setStartTime(4.4);
      message23.getTimeInterval().setEndTime(4.8);
      message23.getQuadrupedStepMessage().setGroundClearance(0.1);
      message23.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message23.getQuadrupedStepMessage().getGoalPosition().set(goalPosition23);

      QuadrupedTimedStepMessage message24 = new QuadrupedTimedStepMessage();
      message24.getTimeInterval().setStartTime(4.6);
      message24.getTimeInterval().setEndTime(5.0);
      message24.getQuadrupedStepMessage().setGroundClearance(0.1);
      message24.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message24.getQuadrupedStepMessage().getGoalPosition().set(goalPosition24);





      QuadrupedTimedStepMessage message25 = new QuadrupedTimedStepMessage();
      message25.getTimeInterval().setStartTime(4.8);
      message25.getTimeInterval().setEndTime(5.2);
      message25.getQuadrupedStepMessage().setGroundClearance(0.1);
      message25.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message25.getQuadrupedStepMessage().getGoalPosition().set(goalPosition25);

      QuadrupedTimedStepMessage message26 = new QuadrupedTimedStepMessage();
      message26.getTimeInterval().setStartTime(5.0);
      message26.getTimeInterval().setEndTime(5.4);
      message26.getQuadrupedStepMessage().setGroundClearance(0.1);
      message26.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message26.getQuadrupedStepMessage().getGoalPosition().set(goalPosition26);

      QuadrupedTimedStepMessage message27 = new QuadrupedTimedStepMessage();
      message27.getTimeInterval().setStartTime(5.2);
      message27.getTimeInterval().setEndTime(5.6);
      message27.getQuadrupedStepMessage().setGroundClearance(0.1);
      message27.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message27.getQuadrupedStepMessage().getGoalPosition().set(goalPosition27);

      QuadrupedTimedStepMessage message28 = new QuadrupedTimedStepMessage();
      message28.getTimeInterval().setStartTime(5.4);
      message28.getTimeInterval().setEndTime(5.8);
      message28.getQuadrupedStepMessage().setGroundClearance(0.1);
      message28.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message28.getQuadrupedStepMessage().getGoalPosition().set(goalPosition28);




      QuadrupedTimedStepMessage message29 = new QuadrupedTimedStepMessage();
      message29.getTimeInterval().setStartTime(5.6);
      message29.getTimeInterval().setEndTime(6.0);
      message29.getQuadrupedStepMessage().setGroundClearance(0.1);
      message29.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_LEFT);
      message29.getQuadrupedStepMessage().getGoalPosition().set(goalPosition29);

      QuadrupedTimedStepMessage message30 = new QuadrupedTimedStepMessage();
      message30.getTimeInterval().setStartTime(5.8);
      message30.getTimeInterval().setEndTime(6.2);
      message30.getQuadrupedStepMessage().setGroundClearance(0.1);
      message30.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_RIGHT);
      message30.getQuadrupedStepMessage().getGoalPosition().set(goalPosition30);

      QuadrupedTimedStepMessage message31 = new QuadrupedTimedStepMessage();
      message31.getTimeInterval().setStartTime(6.0);
      message31.getTimeInterval().setEndTime(6.4);
      message31.getQuadrupedStepMessage().setGroundClearance(0.1);
      message31.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.FRONT_RIGHT);
      message31.getQuadrupedStepMessage().getGoalPosition().set(goalPosition31);

      QuadrupedTimedStepMessage message32 = new QuadrupedTimedStepMessage();
      message32.getTimeInterval().setStartTime(6.2);
      message32.getTimeInterval().setEndTime(6.6);
      message32.getQuadrupedStepMessage().setGroundClearance(0.1);
      message32.getQuadrupedStepMessage().setRobotQuadrant(QuadrupedStepMessage.HIND_LEFT);
      message32.getQuadrupedStepMessage().getGoalPosition().set(goalPosition32);








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
      steps.add(message12);

      steps.add(message13);
      steps.add(message14);
      steps.add(message15);
      steps.add(message16);

      steps.add(message17);
      steps.add(message18);
      steps.add(message19);
      steps.add(message20);

      steps.add(message21);
      steps.add(message22);
      steps.add(message23);
      steps.add(message24);

      steps.add(message25);
      steps.add(message26);
      steps.add(message27);
      steps.add(message28);

      steps.add(message29);
      steps.add(message30);
      steps.add(message31);
      steps.add(message32);


      return steps;
   }
}
