package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.WholeBodyPoseValidityTester;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class WholeBodyPoseValidityTesterTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()) // set
      //if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.close();
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface envrionment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }
   
   @Test
   public void testAPose() throws SimulationExceededMaximumTimeException, IOException
   {
      //ThreadTools.sleep(10000);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();
      
      
      PrintTools.info("Initiate Behavior");
      WholeBodyPoseValidityTester tester = new WholeBodyPoseValidityTester(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                           drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                           drcBehaviorTestHelper.getSDFFullRobotModel());
      PrintTools.info("Success to initiate Behavior");
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 0.001, new Point3D(0.6, -0.35, 1.2), new Quaternion(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
      
      drcBehaviorTestHelper.dispatchBehavior(tester);
      tester.setWholeBodyPose(wholebodyTrajectoryMessage);
      
      PrintTools.info("Start Yo Time "+ drcBehaviorTestHelper.getYoTime());
      
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.001);
      int cnt = 0;
      while(true)
      {
         cnt++;
         ThreadTools.sleep(1);
         PrintTools.info(""+cnt+" "+ drcBehaviorTestHelper.getYoTime() +" SQ "+ tester.getOutputStatus().getSolutionQuality() + " Hand "+ tester.getFullHumanoidRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM03()
                         +" "+ tester.getFullHumanoidRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM13()
                         +" "+ tester.getFullHumanoidRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM23());
         
         if(cnt > 1000)
         {
            //PrintTools.info("End Yo Time "+ drcBehaviorTestHelper.getYoTime());
            
            break;
         }
      }
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(2.4);
      

//      handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 0.001, new Point3D(0.6, -0.35, 1.5), new Quaternion(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
//      wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
//      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
//      
//      tester.setWholeBodyPose(wholebodyTrajectoryMessage);
//      drcBehaviorTestHelper.dispatchBehavior(tester);

      
   }
   
   
   
   
   
  

   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, true);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
   }
}
