package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ClearDelayQueueMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class EndToEndClearDelayQueueMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
   private static final DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testClearingQueue() throws SimulationExceededMaximumTimeException
   {
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(), environment, simulationTestingParameters);
      factory.setStartingLocationOffset(location.getStartingLocationOffset());
      simulationTestHelper = factory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      setupCamera(simulationTestHelper);
      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateAndWait(0.1));

      String handName = simulationTestHelper.getControllerFullRobotModel().getHand(RobotSide.LEFT).getName();
      YoVariable footsteps = simulationTestHelper.findVariable(WalkingMessageHandler.class.getSimpleName(), "currentNumberOfFootsteps");
      YoVariable handTrajectoryPoints = simulationTestHelper.findVariable(handName + "TaskspaceControlModule", handName + "TaskspaceNumberOfPoints");

      // send hand trajectory and footstep list
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(RobotSide.LEFT.toByte());
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      for (int i = 0; i < 10; i++)
      {
         handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage((double) i, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
         footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(), new Quaternion()));
      }
      handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionDelayTime(0.1);
      footstepDataListMessage.getQueueingProperties().setExecutionDelayTime(0.1);
      simulationTestHelper.publishToController(handTrajectoryMessage);
      simulationTestHelper.publishToController(footstepDataListMessage);
      assertTrue(simulationTestHelper.simulateAndWait(0.05));

      assertEquals(0, (int) footsteps.getValueAsLongBits());
      assertEquals(0, (int) handTrajectoryPoints.getValueAsLongBits());

      // clear hand trajectory
      ClearDelayQueueMessage clearHandTrajectory = HumanoidMessageTools.createClearDelayQueueMessage(HandTrajectoryMessage.class);
      simulationTestHelper.publishToController(clearHandTrajectory);
      assertTrue(simulationTestHelper.simulateAndWait(0.1));

      assertEquals(10, (int) footsteps.getValueAsLongBits());
      assertEquals(0, (int) handTrajectoryPoints.getValueAsLongBits());
   }

   private static void setupCamera(SCS2AvatarTestingSimulation simulationTestHelper)
   {
      OffsetAndYawRobotInitialSetup startingLocationOffset = location.getStartingLocationOffset();
      Point3D cameraFocus = new Point3D(startingLocationOffset.getAdditionalOffset());
      cameraFocus.addZ(1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(startingLocationOffset.getYaw());
      Point3D cameraPosition = new Point3D(10.0, 5.0, cameraFocus.getZ() + 2.0);
      transform.transform(cameraPosition);
      simulationTestHelper.setCamera(cameraFocus, cameraPosition);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
