package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.*;
import org.junit.After;
import org.junit.Before;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.communication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.communication.QuadrupedNetClassList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public abstract class QuadrupedScriptedFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
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

   public void testScriptedFlatGroundWalking() throws IOException, BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
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

      QuadrupedRequestedControllerStateMessage controllerMessage = new QuadrupedRequestedControllerStateMessage();
      controllerMessage.setQuadrupedControllerName(QuadrupedControllerRequestedEvent.REQUEST_STEPPING.toByte());
      packetCommunicator.send(controllerMessage);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      List<QuadrupedTimedStepMessage> steps = getSteps();
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

      // check robot is still upright and walked forward
      Point3D expectedFinalPlanarPosition = getFinalPlanarPosition();
      assertEquals(variables.getRobotBodyX().getDoubleValue(), expectedFinalPlanarPosition.getX(), 0.1);
      assertEquals(variables.getRobotBodyY().getDoubleValue(), expectedFinalPlanarPosition.getY(), 0.1);
      assertEquals(variables.getRobotBodyYaw().getDoubleValue(), expectedFinalPlanarPosition.getZ(), 0.1);
      assertTrue(variables.getRobotBodyZ().getDoubleValue() > 0.0);
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
