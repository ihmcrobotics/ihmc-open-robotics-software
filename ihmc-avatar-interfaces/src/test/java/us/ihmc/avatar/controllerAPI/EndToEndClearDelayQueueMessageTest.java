package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.ClearDelayQueueMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class EndToEndClearDelayQueueMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
   private static final DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testClearingQueue() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.setStartingLocation(location);
      drcSimulationTestHelper.createSimulation("Test");
      setupCamera(drcSimulationTestHelper);
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      String handName = drcSimulationTestHelper.getControllerFullRobotModel().getHand(RobotSide.LEFT).getName();
      YoVariable<?> footsteps = scs.getVariable(WalkingMessageHandler.class.getSimpleName(), "currentNumberOfFootsteps");
      YoVariable<?> handTrajectoryPoints = scs.getVariable(handName + "TaskspaceControlModule", handName + "TaskspaceNumberOfPoints");

      // send hand trajectory and footstep list
      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.LEFT, 10);
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      for (int i = 0; i < 10; i++)
      {
         handTrajectoryMessage.getSe3Trajectory().setTrajectoryPoint(i, i, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D(), ReferenceFrame.getWorldFrame());
         footstepDataListMessage.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(), new Quaternion()));
      }
      handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionDelayTime(0.1);
      footstepDataListMessage.setExecutionDelayTime(0.1);
      drcSimulationTestHelper.send(handTrajectoryMessage);
      drcSimulationTestHelper.send(footstepDataListMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05));

      assertEquals(0, (int) footsteps.getValueAsLongBits());
      assertEquals(0, (int) handTrajectoryPoints.getValueAsLongBits());

      // clear hand trajectory
      ClearDelayQueueMessage clearHandTrajectory = HumanoidMessageTools.createClearDelayQueueMessage(HandTrajectoryMessage.class);
      drcSimulationTestHelper.send(clearHandTrajectory);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));

      assertEquals(10, (int) footsteps.getValueAsLongBits());
      assertEquals(0, (int) handTrajectoryPoints.getValueAsLongBits());
   }

   private static void setupCamera(DRCSimulationTestHelper drcSimulationTestHelper)
   {
      OffsetAndYawRobotInitialSetup startingLocationOffset = location.getStartingLocationOffset();
      Point3D cameraFocus = new Point3D(startingLocationOffset.getAdditionalOffset());
      cameraFocus.addZ(1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(startingLocationOffset.getYaw());
      Point3D cameraPosition = new Point3D(10.0, 5.0, cameraFocus.getZ() + 2.0);
      transform.transform(cameraPosition);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFocus, cameraPosition);
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
