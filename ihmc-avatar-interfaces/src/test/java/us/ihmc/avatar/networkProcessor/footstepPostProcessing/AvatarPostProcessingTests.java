package us.ihmc.avatar.networkProcessor.footstepPostProcessing;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingToolboxModule;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public abstract class AvatarPostProcessingTests implements MultiRobotTestInterface
{
   private static final boolean keepSCSUp = true;

   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   private IHMCROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;
   private IHMCROS2Publisher<FootstepPostProcessingPacket> postProcessingRequestPublisher;
   private IHMCROS2Publisher<FootstepPlannerParametersPacket> planningParametersPublisher;
   private IHMCROS2Publisher<FootstepPostProcessingParametersPacket> postProcessingParametersPublisher;
   private IHMCROS2Publisher<ToolboxStateMessage> planningToolboxPublisher;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus;
   private AtomicReference<FootstepPostProcessingPacket> postProcessingOutputStatus;

   private FootstepPlanningToolboxModule footstepToolboxModule;
   private FootstepPlanPostProcessingToolboxModule postProcessingToolboxModule;

   private FootstepPlannerParametersBasics footstepPlannerParameters;

   @BeforeEach
   public void showMemoryUsageBeforeTest() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      footstepToolboxModule = new FootstepPlanningToolboxModule(getRobotModel(), null, false, DomainFactory.PubSubImplementation.INTRAPROCESS);
      postProcessingToolboxModule = new FootstepPlanPostProcessingToolboxModule(getRobotModel(), null, false, DomainFactory.PubSubImplementation.INTRAPROCESS);

      plannerOutputStatus = new AtomicReference<>();
      postProcessingOutputStatus = new AtomicReference<>();

      Ros2Node ros2Node = drcSimulationTestHelper.getRos2Node();

      String robotName = robotModel.getSimpleRobotName();
      MessageTopicNameGenerator footstepPlannerSubGenerator = FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator footstepPlannerPubGenerator = FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator postProcessingPubGenerator = getTopicNameGenerator(robotName, FootstepPlanPostProcessingToolboxModule.moduleName, ROS2Tools.ROS2TopicQualifier.OUTPUT);
      MessageTopicNameGenerator postProcessingSubGenerator = getTopicNameGenerator(robotName, FootstepPlanPostProcessingToolboxModule.moduleName, ROS2Tools.ROS2TopicQualifier.INPUT);

      planningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, footstepPlannerSubGenerator);
      postProcessingRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPostProcessingPacket.class, postProcessingSubGenerator);
      postProcessingParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPostProcessingParametersPacket.class, postProcessingSubGenerator);
      planningParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlannerParametersPacket.class, footstepPlannerSubGenerator);
      planningToolboxPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, footstepPlannerSubGenerator);

      drcSimulationTestHelper.createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlannerPubGenerator, plannerOutputStatus::set);
      drcSimulationTestHelper.createSubscriber(FootstepPostProcessingPacket.class, postProcessingPubGenerator, postProcessingOutputStatus::set);

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

//      if (simulationTestingParameters.getKeepSCSUp())
//      {
//         ThreadTools.sleepForever();
//      }

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

      DefaultFootstepPostProcessingParameters parameters = new DefaultFootstepPostProcessingParameters();
      parameters.setSwingOverRegionsProcessingEnabled(true);

      postProcessingParametersPublisher.publish(parameters.getAsPacket());

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

   @Test
   public void testWalkingOnStraightForwardLines() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();

      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      DefaultFootstepPostProcessingParameters parameters = new DefaultFootstepPostProcessingParameters();
      parameters.setFractionLoadIfFootHasFullSupport(0.6);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(0.6);
      parameters.setFractionLoadIfOtherFootHasNoWidth(0.7);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(0.7);

      postProcessingParametersPublisher.publish(parameters.getAsPacket());

      // increase ankle damping to match the real robot better
      YoDouble damping_l_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_akx");
      YoDouble damping_l_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_aky");
      YoDouble damping_r_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_akx");
      YoDouble damping_r_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_aky");
      damping_l_akx.set(1.0);
      damping_l_aky.set(1.0);
      damping_r_akx.set(1.0);
      damping_r_aky.set(1.0);

      SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();
      // get foot states
      for (RobotSide robotSide : RobotSide.values)
      {
         String variableName = robotSide.getCamelCaseNameForStartOfExpression() + "FootCurrentState";
         YoEnum<ConstraintType> footState = (YoEnum<ConstraintType>) drcSimulationTestHelper.getYoVariable(variableName);
         footStates.put(robotSide, footState);
      }



      // setup camera
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      final ContactPointController contactPointController = new ContactPointController(footStates);
      drcSimulationTestHelper.addRobotControllerOnControllerThread(contactPointController);
//      setupSupportViz();

      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double footForwardOffset = steppingParameters.getFootForwardOffset();
      double footBackwardOffset = steppingParameters.getFootBackwardOffset();
      double footWidth = steppingParameters.getFootWidth();
      double toeWidth = steppingParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2D defaultSolePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(soleVertices));
      defaultSolePolygon.update();


      ThreadTools.sleep(1000);


      armsUp();



      double swingDuration = 0.6;
      double transferDuration = 0.5;
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingDuration, transferDuration);

      int numberOfSteps = 2;

      for (int i = 0; i < numberOfSteps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         ArrayList<Point2D> newContactPoints = generateContactPointsForRotatedLineOfContact(0.0, 0.0, 0.0);

         FootstepDataMessage footstepData = new FootstepDataMessage();

         ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
         FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
         placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         placeToStepInWorld.setX(0.3 * i);

         footstepData.getLocation().set(placeToStepInWorld);
         footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide.toByte());
         for (Point2D contactPoint : newContactPoints)
            footstepData.getPredictedContactPoints2d().add().set(contactPoint);

         message.getFootstepDataList().add().set(footstepData);
      }

      RobotSide robotSide = numberOfSteps % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

      FootstepDataMessage footstepData = new FootstepDataMessage();

      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
      placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      placeToStepInWorld.setX(0.3 * (numberOfSteps - 1));

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide.toByte());

      message.getFootstepDataList().add().set(footstepData);

      FramePose3D leftFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPostProcessingPacket postProcessingRequest = new FootstepPostProcessingPacket();
      postProcessingRequest.getFootstepDataList().set(message);
      postProcessingRequest.getLeftFootPositionInWorld().set(leftFootPose.getPosition());
      postProcessingRequest.getLeftFootOrientationInWorld().set(leftFootPose.getOrientation());
      postProcessingRequest.getRightFootPositionInWorld().set(rightFootPose.getPosition());
      postProcessingRequest.getRightFootOrientationInWorld().set(rightFootPose.getOrientation());
      for (Point2DReadOnly vertex : defaultSolePolygon.getVertexBufferView())
      {
         postProcessingRequest.getLeftFootContactPoints2d().add().set(vertex);
         postProcessingRequest.getRightFootContactPoints2d().add().set(vertex);
      }

      postProcessingRequestPublisher.publish(postProcessingRequest);

      double maxTimeToWait = 20.0;
      long startTime = System.nanoTime();
      while (postProcessingOutputStatus.get() == null && Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) < maxTimeToWait)
      {
         ThreadTools.sleep(100);
      }

      if (postProcessingOutputStatus.get() == null)
      {
         fail("Never received an output from the post processor, even after " + maxTimeToWait + " seconds.");
      }

      FootstepDataListMessage footstepDataListMessage = postProcessingOutputStatus.get().getFootstepDataList();

      List<FootstepDataMessage> footsteps = new ArrayList<>(footstepDataListMessage.getFootstepDataList());

      int stepCounter = 0;
      for (RobotSide robotSide1 : RobotSide.values)
      {
         footStates.get(robotSide1).addVariableChangedListener(v -> {
            if (footStates.get(robotSide1).getEnumValue() == ConstraintType.SWING)
            {
               List<Point3D> contactPoints3D = footsteps.remove(stepCounter).getPredictedContactPoints2d();
               if (contactPoints3D.size() < 1)
               {
                  contactPointController.setNewContacts(defaultSolePolygon.getVertexBufferView(), robotSide1, true);
               }
               else
               {
                  List<Point2D> newContactPoints = contactPoints3D.stream().map(Point2D::new).collect(Collectors.toList());
                  contactPointController.setNewContacts(newContactPoints, robotSide1, true);
               }

            }
         });
      }

      drcSimulationTestHelper.publishToController(footstepDataListMessage);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions((swingDuration + transferDuration) * numberOfSteps + 5.0);
      assertTrue(success);
   }

   private static final double[] rightHandStraightSideJointAngles = new double[] {-0.5067668142160446, -0.3659876546358431, 1.7973796317575155, -1.2398714600960365, -0.005510224629709242, 0.6123343067479899, 0.12524505635696856};
   private static final double[] leftHandStraightSideJointAngles = new double[] {0.61130147334225, 0.22680071472282162, 1.6270339908033258, 1.2703560974484844, 0.10340544060719102, -0.6738299572358809, 0.13264785356924128};
   private static final SideDependentList<double[]> straightArmConfigs = new SideDependentList<>();
   static
   {
      straightArmConfigs.put(RobotSide.LEFT, leftHandStraightSideJointAngles);
      straightArmConfigs.put(RobotSide.RIGHT, rightHandStraightSideJointAngles);
   }

   private void armsUp() throws SimulationExceededMaximumTimeException
   {
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      // bring the arms in a stretched position
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.setRobotSide(robotSide.toByte());
         double[] armConfig = straightArmConfigs.get(robotSide);
         for (int i = 0; i < armConfig.length; i++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
            trajectoryPoint.setPosition(armConfig[i]);
            trajectoryPoint.setTime(0.5);
            OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
            jointTrajectory.getTrajectoryPoints().add().set(trajectoryPoint);
            armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(jointTrajectory);
         }
         drcSimulationTestHelper.publishToController(armTrajectoryMessage);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
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

      FramePose3D leftFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPostProcessingPacket postProcessingRequest = new FootstepPostProcessingPacket();
      postProcessingRequest.getFootstepDataList().set(plannerOutputStatus.get().getFootstepDataList());
      postProcessingRequest.getPlanarRegionsList().set(plannerOutputStatus.get().getPlanarRegionsList());
      postProcessingRequest.getLeftFootPositionInWorld().set(leftFootPose.getPosition());
      postProcessingRequest.getLeftFootOrientationInWorld().set(leftFootPose.getOrientation());
      postProcessingRequest.getRightFootPositionInWorld().set(rightFootPose.getPosition());
      postProcessingRequest.getRightFootOrientationInWorld().set(rightFootPose.getOrientation());

      postProcessingRequestPublisher.publish(postProcessingRequest);

      startTime = System.nanoTime();
      while (postProcessingOutputStatus.get() == null && Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) < maxTimeToWait)
      {
         ThreadTools.sleep(100);
      }

      if (postProcessingOutputStatus.get() == null)
      {
         fail("Never received an output from the post processor, even after " + maxTimeToWait + " seconds.");
      }

      FootstepDataListMessage footstepDataListMessage = postProcessingOutputStatus.get().getFootstepDataList();

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      double stepTime = footstepDataListMessage.getDefaultSwingDuration() + footstepDataListMessage.getDefaultTransferDuration();
      if (stepTime < 0.5)
      {
         WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
         stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      }
      double simulationTime = 2.0 + stepTime * footstepDataListMessage.getFootstepDataList().size();

      boolean success =  drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(request.getGoalPositionInWorld());
      center.addZ(0.7);

      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   private class ContactPointController implements RobotController
   {
      private List<? extends Point2DReadOnly> newContactPoints = null;
      private RobotSide robotSide = null;

      private AtomicBoolean setNewContactPoints = new AtomicBoolean(false);
      private boolean setOnStep = false;

      private SideDependentList<YoEnum<ConstraintType>> footStates;

      public ContactPointController(SideDependentList<YoEnum<ConstraintType>> footStates)
      {
         this.footStates = footStates;
      }

      /**
       * Changes the foot contact points of the robot.
       * The contact points can be changed immediately or when the foot is in swing.
       *
       * @param newContactPoints
       * @param robotSide
       * @param setOnStep
       */
      public void setNewContacts(List<? extends Point2DReadOnly> newContactPoints, RobotSide robotSide, boolean setOnStep)
      {
         if (setNewContactPoints.get())
         {
            System.err.println("New contact points are already waiting to be set.");
            return;
         }

         this.newContactPoints = newContactPoints;
         this.robotSide = robotSide;
         this.setOnStep = setOnStep;

         setNewContactPoints.set(true);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return null;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }

      @Override
      public void doControl()
      {
         if (setNewContactPoints.get())
         {
            if (!setOnStep)
               setNewContacts();
            else if (footStates.get(robotSide).getEnumValue() == ConstraintType.SWING)
               setNewContacts();
         }
      }

      private void setNewContacts()
      {
         String footJointName = drcSimulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getParentJoint().getName();
         HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

         int pointIndex = 0;
         ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

         for (GroundContactPoint point : allGroundContactPoints)
         {
            Joint parentJoint = point.getParentJoint();

            if (parentJoint.getName().equals(footJointName))
            {
               Point2DReadOnly newContactPoint = newContactPoints.get(pointIndex);

               point.setIsInContact(false);
               Vector3D offset = new Vector3D();
               point.getOffset(offset);

               offset.setX(newContactPoint.getX());
               offset.setY(newContactPoint.getY());

               point.setOffsetJoint(offset);
               pointIndex++;
            }
         }

//         if (footContactsInAnkleFrame != null)
//         {
//            footContactsInAnkleFrame.set(robotSide, newContactPoints);
//         }

         setNewContactPoints.set(false);
      }

   }

   private ArrayList<Point2D> generateContactPointsForRotatedLineOfContact(double angle, double xLine, double yLine)
   {
      double lineWidth = 0.01;

      // build default foot polygon:
      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double footForwardOffset = steppingParameters.getFootForwardOffset();
      double footBackwardOffset = steppingParameters.getFootBackwardOffset();
      double footWidth = steppingParameters.getFootWidth();
      double toeWidth = steppingParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2D solePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(soleVertices));
      solePolygon.update();

      // shrink polygon and project line origin inside
      ConvexPolygon2D shrunkSolePolygon = new ConvexPolygon2D();
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      shrinker.scaleConvexPolygon(solePolygon, lineWidth/2.0 + (footWidth-toeWidth)/2.0, shrunkSolePolygon);

      Point2D lineOrigin = new Point2D(xLine, yLine);
      shrunkSolePolygon.orthogonalProjection(lineOrigin);

      // transform line and compute intersections with default foot polygon
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(angle);
      transform.setTranslation(lineOrigin.getX(), lineOrigin.getY(), 0.0);

      Line2D line = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      line.applyTransform(transform);

      line.shiftToLeft(lineWidth/2.0);
      Point2DBasics[] leftIntersections = solePolygon.intersectionWith(line);
      line.shiftToRight(lineWidth);
      Point2DBasics[] rightIntersections = solePolygon.intersectionWith(line);

      ArrayList<Point2D> ret = new ArrayList<Point2D>();
      ret.add(new Point2D(leftIntersections[0]));
      ret.add(new Point2D(leftIntersections[1]));
      ret.add(new Point2D(rightIntersections[0]));
      ret.add(new Point2D(rightIntersections[1]));
      return ret;
   }
}
