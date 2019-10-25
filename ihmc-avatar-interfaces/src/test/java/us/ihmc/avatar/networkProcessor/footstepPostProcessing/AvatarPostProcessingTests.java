package us.ihmc.avatar.networkProcessor.footstepPostProcessing;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.ToolboxStateMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingToolboxModule;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
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
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

public abstract class AvatarPostProcessingTests implements MultiRobotTestInterface
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   private IHMCROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;
   private IHMCROS2Publisher<FootstepPlanningToolboxOutputStatus> postProcessingRequestPublisher;
   private IHMCROS2Publisher<FootstepPlannerParametersPacket> planningParametersPublisher;
   private IHMCROS2Publisher<ToolboxStateMessage> planningToolboxPublisher;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> postProcessingOutputStatus;

   private FootstepPlanningToolboxModule footstepToolboxModule;
   private FootstepPlanPostProcessingToolboxModule postProcessingToolboxModule;

   private FootstepPlannerParametersBasics footstepPlannerParameters;

   @BeforeEach
   public void showMemoryUsageBeforeTest() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableFootstepPlanningToolbox(true);
      networkModuleParameters.enableNetworkProcessor(true);
      networkModuleParameters.enableLocalControllerCommunicator(true);

      footstepToolboxModule = new FootstepPlanningToolboxModule(getRobotModel(), null, true, DomainFactory.PubSubImplementation.INTRAPROCESS);
      postProcessingToolboxModule = new FootstepPlanPostProcessingToolboxModule(getRobotModel(), null, true, DomainFactory.PubSubImplementation.INTRAPROCESS);

      drcSimulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);

      plannerOutputStatus = new AtomicReference<>();
      postProcessingOutputStatus = new AtomicReference<>();

      Ros2Node ros2Node = drcSimulationTestHelper.getRos2Node();

      String robotName = robotModel.getSimpleRobotName();
      MessageTopicNameGenerator footstepPlannerSubGenerator = FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator footstepPlannerPubGenerator = FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator postProcessingPubGenerator = getTopicNameGenerator(robotName, FootstepPlanPostProcessingToolboxModule.moduleName, ROS2Tools.ROS2TopicQualifier.OUTPUT);
      MessageTopicNameGenerator postProcessingSubGenerator = getTopicNameGenerator(robotName, FootstepPlanPostProcessingToolboxModule.moduleName, ROS2Tools.ROS2TopicQualifier.INPUT);

      planningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, footstepPlannerSubGenerator);
      postProcessingRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningToolboxOutputStatus.class, postProcessingSubGenerator);
      planningParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlannerParametersPacket.class, footstepPlannerSubGenerator);
      planningToolboxPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, footstepPlannerSubGenerator);

      drcSimulationTestHelper.createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlannerPubGenerator, plannerOutputStatus::set);
      drcSimulationTestHelper.createSubscriber(FootstepPlanningToolboxOutputStatus.class, postProcessingPubGenerator, postProcessingOutputStatus::set);

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

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
      postProcessingRequestPublisher = null;
      planningParametersPublisher = null;
      planningToolboxPublisher = null;
      plannerOutputStatus = null;
      postProcessingOutputStatus = null;

      footstepToolboxModule.destroy();
      postProcessingToolboxModule.destroy();
      footstepToolboxModule = null;
      postProcessingToolboxModule = null;

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

      footstepPlannerParameters.setMaximumStepZ(height + 0.05);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      FootstepPlannerParametersPacket parametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(parametersPacket, footstepPlannerParameters);
      planningParametersPublisher.publish(parametersPacket);

      PoseReferenceFrame startingFrame = new PoseReferenceFrame("startingFrame", ReferenceFrame.getWorldFrame());
      startingFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), startingLocation.getAdditionalOffset()));
      startingFrame.setOrientationAndUpdate(new Quaternion(startingLocation.getYaw(), 0.0, 0.0));

      FramePose3D goalPose = new FramePose3D(startingFrame);
      goalPose.setPosition(1.0, 0.0, 0.0);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = getRequest(drcSimulationTestHelper.getControllerFullRobotModel(), blockEnvironment.getPlanarRegionsList(), goalPose);

      runTest(request);
   }

   @Disabled
   @Test
   public void testSwingOverPlanarRegions() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();

      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();

      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.createSimulation(className);

      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);

      footstepPlannerParameters.setMaximumStepReach(0.6);
      footstepPlannerParameters.setMinimumStepWidth(0.05);
      footstepPlannerParameters.setMaximumStepWidth(0.35);
      footstepPlannerParameters.setBodyBoxBaseZ(0.4);
      footstepPlannerParameters.setCheckForBodyBoxCollisions(false);
      footstepPlannerParameters.setCheckForPathCollisions(false);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      FootstepPlannerParametersPacket parametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(parametersPacket, footstepPlannerParameters);
      planningParametersPublisher.publish(parametersPacket);

      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(2.0, 0.0, 0.0);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket requestPacket = getRequest(drcSimulationTestHelper.getControllerFullRobotModel(), environment.getPlanarRegionsList(), goalPose);
      runTest(requestPacket);
   }


   private static FootstepPlanningRequestPacket getRequest(FullHumanoidRobotModel fullRobotModel, PlanarRegionsList planarRegionsList, FramePose3D goalPose)
   {
      FramePose3D leftFoot = new FramePose3D(fullRobotModel.getSoleFrame(RobotSide.LEFT));
      leftFoot.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket();
      request.setInitialStanceRobotSide(FootstepPlanningRequestPacket.ROBOT_SIDE_LEFT);
      request.getStanceFootPositionInWorld().set(leftFoot.getPosition());
      request.getStanceFootOrientationInWorld().set(leftFoot.getOrientation());

      request.getGoalPositionInWorld().set(goalPose.getPosition());
      request.getGoalOrientationInWorld().set(goalPose.getOrientation());

      request.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));

      request.setRequestedFootstepPlannerType(FootstepPlannerType.A_STAR.toByte());

      return request;
   }


   private void runTest(FootstepPlanningRequestPacket request) throws SimulationExceededMaximumTimeException
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoFramePoint3D goalPosition = new YoFramePoint3D("goalPosition", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition goalGraphic = new YoGraphicPosition("goalGraphic", goalPosition, 0.05, YoAppearance.Green());
      goalPosition.set(request.getGoalPositionInWorld());
      yoGraphicsListRegistry.registerYoGraphic("Test", goalGraphic);
      drcSimulationTestHelper.addChildRegistry(registry);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);

      planningRequestPublisher.publish(request);
      ThreadTools.sleep(100);

      planningToolboxPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      double maxTimeToWait = 20.0;
      long startTime = System.nanoTime();
      while (plannerOutputStatus.get() == null && Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) < maxTimeToWait)
      {
         ThreadTools.sleep(100);
      }

      if (plannerOutputStatus.get() == null)
      {
         fail("Never received an output from the footstep planner, even after 20 seconds.");
      }

      postProcessingRequestPublisher.publish(plannerOutputStatus.get());

      startTime = System.nanoTime();
      while (postProcessingOutputStatus.get() == null && Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) < maxTimeToWait)
      {
         ThreadTools.sleep(100);
      }

      if (postProcessingOutputStatus.get() == null)
      {
         fail("Never received an output from the post processor, even after " + maxTimeToWait + " seconds.");
      }


      drcSimulationTestHelper.publishToController(postProcessingOutputStatus.get().getFootstepDataList());

      boolean success =  drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(request.getGoalPositionInWorld());
      center.addZ(0.7);

      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }
}
