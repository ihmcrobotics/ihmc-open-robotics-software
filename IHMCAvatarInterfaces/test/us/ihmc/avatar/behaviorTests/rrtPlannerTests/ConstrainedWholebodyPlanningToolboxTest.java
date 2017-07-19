package us.ihmc.avatar.behaviorTests.rrtPlannerTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.ConstrainedWholebodyPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholebodyPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class ConstrainedWholebodyPlanningToolboxTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private ConstrainedWholebodyPlanningToolboxModule cwbPlanningToolboxModule;
   
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
   
   private void setupCWBPlanningToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false);
      cwbPlanningToolboxModule = new ConstrainedWholebodyPlanningToolboxModule(robotModel, fullRobotModel, null, false);
            
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.CWB_PLANNING_TOOLBOX_MODULE_PORT, PacketDestination.CWB_PLANNING_TOOLBOX_MODULE);
      
   }
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");      
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (cwbPlanningToolboxModule != null)
      {
         cwbPlanningToolboxModule.destroy();
         cwbPlanningToolboxModule = null;
      }
      
      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupCWBPlanningToolboxModule();
   }
   
   @Test
   public void testForToolboxMessage() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      ToolboxStateMessage toolboxMessage;
      
      drcBehaviorTestHelper.updateRobotModel();       
      System.out.println("Start");
      
      System.out.println("Send wake up "+drcBehaviorTestHelper.getYoTime());
      
      toolboxMessage = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxMessage.setDestination(PacketDestination.CWB_PLANNING_TOOLBOX_MODULE);
      toolboxCommunicator.send(toolboxMessage);
            
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("Send input "+drcBehaviorTestHelper.getYoTime());
      
      ConstrainedWholebodyPlanningRequestPacket requestPacket = new ConstrainedWholebodyPlanningRequestPacket();   
      requestPacket.setTempValue(3.1);
      toolboxCommunicator.send(requestPacket);
      
      System.out.println("Send input Done "+drcBehaviorTestHelper.getYoTime());
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

}
