package us.ihmc.avatar.networkProcessor.footstepPostProcessing;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.ToolboxStateMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

public abstract class AvatarLargeStepDownTests implements MultiRobotTestInterface
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   private IHMCROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;
   private IHMCROS2Publisher<ToolboxStateMessage> planningToolboxPublisher;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus;


   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);


      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableFootstepPlanningToolbox(true);

      drcSimulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);

      plannerOutputStatus = new AtomicReference<>();

      Ros2Node ros2Node = drcSimulationTestHelper.getRos2Node();

      String robotName = robotModel.getSimpleRobotName();
      MessageTopicNameGenerator footstepPlannerSubGenerator = FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator footstepPlannerPubGenerator = FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName);

      planningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, footstepPlannerSubGenerator);
      planningToolboxPublisher = drcSimulationTestHelper.createPublisher(ToolboxStateMessage.class, footstepPlannerSubGenerator);

      drcSimulationTestHelper.createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlannerPubGenerator, plannerOutputStatus::set);

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
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

      planningRequestPublisher = null;
      plannerOutputStatus = null;

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testWalkingOffOfMediumPlatform() throws SimulationExceededMaximumTimeException
   {
      double height = 0.3;
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
      startingLocation.addAdditionalOffset(new Vector3D(0.0, 0.0, height));

      BlockEnvironment blockEnvironment = new BlockEnvironment(1.0, 1.0, height);
      drcSimulationTestHelper.setTestEnvironment(blockEnvironment);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOntoMediumPlatformToesTouchingTest");

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      FramePose3D leftFoot = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      leftFoot.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket();
      request.setInitialStanceRobotSide(FootstepPlanningRequestPacket.ROBOT_SIDE_LEFT);
      request.getStanceFootPositionInWorld().set(leftFoot.getPosition());
      request.getStanceFootOrientationInWorld().set(leftFoot.getOrientation());

      PoseReferenceFrame startingFrame = new PoseReferenceFrame("startingFrame", ReferenceFrame.getWorldFrame());
      startingFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), startingLocation.getAdditionalOffset()));
      startingFrame.setOrientationAndUpdate(new Quaternion(startingLocation.getYaw(), 0.0, 0.0));

      FramePose3D goalPose = new FramePose3D(startingFrame);
      goalPose.setPosition(1.0, 0.0, 0.0);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      request.getGoalPositionInWorld().set(goalPose.getPosition());
      request.getGoalOrientationInWorld().set(goalPose.getOrientation());

      request.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(blockEnvironment.getPlanarRegionsList()));

      planningRequestPublisher.publish(request);
      ThreadTools.sleep(100);

      planningToolboxPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);


      double maxTimeToWait = 20.0;
      long startTime = System.nanoTime();
      while (plannerOutputStatus.get() == null && Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) < maxTimeToWait)
      {
         ThreadTools.sleep(100);
      }

      if (plannerOutputStatus.get() == null)
      {
         fail("Never received an output, even after 20 seconds.");
      }

      drcSimulationTestHelper.publishToController(plannerOutputStatus.get().getFootstepDataList());

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

}
