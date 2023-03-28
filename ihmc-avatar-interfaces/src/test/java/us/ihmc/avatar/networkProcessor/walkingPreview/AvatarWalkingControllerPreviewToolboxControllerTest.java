package us.ihmc.avatar.networkProcessor.walkingPreview;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Random;
import java.util.stream.Stream;

import org.apache.commons.lang3.mutable.MutableObject;
import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage;
import toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerOutputBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

@Tag("humanoid-toolbox")
public abstract class AvatarWalkingControllerPreviewToolboxControllerTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double EPSILON = 1.0e-12;
   private static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private CommandInputManager toolboxInputManager;
   private StatusMessageOutputManager toolboxOutputManager;
   private YoRegistry toolboxMainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private WalkingControllerPreviewToolboxController toolboxController;

   private YoBoolean enableToolboxUpdater;
   private YoBoolean pauseToolboxUpdater;
   private YoBoolean initializationSucceeded;

   private Robot ghost;
   private Controller toolboxUpdater;

   public void setup(double integrationDT)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      toolboxMainRegistry = new YoRegistry("toolboxMain");
      enableToolboxUpdater = new YoBoolean("enableToolboxUpdater", toolboxMainRegistry);
      pauseToolboxUpdater = new YoBoolean("pauseToolboxUpdater", toolboxMainRegistry);
      initializationSucceeded = new YoBoolean("initializationSucceeded", toolboxMainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();
      toolboxInputManager = new CommandInputManager(WalkingControllerPreviewToolboxModule.supportedCommands());
      toolboxOutputManager = new StatusMessageOutputManager(WalkingControllerPreviewToolboxModule.supportedStatus());
      toolboxController = new WalkingControllerPreviewToolboxController(robotModel,
                                                                        integrationDT,
                                                                        toolboxInputManager,
                                                                        toolboxOutputManager,
                                                                        yoGraphicsListRegistry,
                                                                        toolboxMainRegistry);
   }

   private void createGhostRobot()
   {
      DRCRobotModel robotModel = getRobotModel();
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      ghostRobotDefinition.setName("Ghost");
      ghostRobotDefinition.ignoreAllJoints();
      RobotDefinitionTools.setRobotDefinitionMaterial(ghostRobotDefinition, ghostMaterial);
      ghost = new Robot(ghostRobotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
   }

   @AfterEach
   public void teardown()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testWalkingPreviewAlone()
   {
      double dt = 0.02;
      setup(dt);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      RobotDefinition robotDefinition = getRobotModel().getRobotDefinition();
      robotDefinition.ignoreAllJoints();
      getRobotModel().getDefaultRobotInitialSetup(0.0, 0.0).initializeRobotDefinition(robotDefinition);
      Robot robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      toolboxUpdater = createToolboxUpdater(robot);
      robot.addController(toolboxUpdater);

      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.addRobot(robot);
      scs.setDT(toolboxController.getIntegrationDT());
      scs.initializeBufferRecordTickPeriod(1);

      simulationTestHelper = new SCS2AvatarTestingSimulation(scs, getRobotModel(), null, yoGraphicsListRegistry, simulationTestingParameters);
      simulationTestHelper.start();

      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelAtInitialConfiguration(2.0);
      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(fullRobotModelAtInitialConfiguration);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      SideDependentList<Pose3DReadOnly> footPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(fullRobotModelAtInitialConfiguration.getSoleFrame(robotSide));
         footPose.changeFrame(worldFrame);
         footPoses.put(robotSide, footPose);
      }

      WalkingControllerPreviewInputMessage input = new WalkingControllerPreviewInputMessage();
      Object<FootstepDataMessage> footstepDataList = input.getFootsteps().getFootstepDataList();
      int numberOfFootsteps = 10;

      for (int i = 0; i < numberOfFootsteps; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         footstepDataList.add()
                         .set(HumanoidMessageTools.createFootstepDataMessage(side, footPoses.get(side).getPosition(), footPoses.get(side).getOrientation()));
      }

      toolboxInputManager.submitMessage(input);

      enableToolboxUpdater.set(true);

      MutableObject<WalkingControllerPreviewOutputMessage> latestOutput = new MutableObject<>();
      toolboxOutputManager.attachStatusMessageListener(WalkingControllerPreviewOutputMessage.class, latestOutput::setValue);
      int expectedNumberOfFrames = 0;

      for (int i = 0; i < 1000; i++)
      {
         simulationTestHelper.simulateOneTickNow();

         if (toolboxController.isWalkingControllerResetDone())
            expectedNumberOfFrames++;

         if (toolboxController.isDone())
            break;
      }

      enableToolboxUpdater.set(false);

      assertNotNull(latestOutput.getValue());
      double expectedFrameDT = dt;

      if (expectedNumberOfFrames > MessageTools.WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES)
      {
         expectedFrameDT = dt * (double) expectedNumberOfFrames / (double) MessageTools.WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES;
         expectedNumberOfFrames = MessageTools.WALKING_PREVIEW_MAX_NUMBER_OF_FRAMES;
      }
      assertEquals(expectedFrameDT, latestOutput.getValue().getFrameDt(), EPSILON);
      int actualNumberOfFrames = latestOutput.getValue().getRobotConfigurations().size();
      assertEquals(expectedNumberOfFrames, actualNumberOfFrames);
   }

   @SuppressWarnings("unchecked")
   @Test
   public void testStepsInPlacePreviewAtControllerDT()
   {
      setup(getRobotModel().getControllerDT());
      createGhostRobot();
      simulationTestingParameters.setRunMultiThreaded(false);
      simulationTestingParameters.setUsePefectSensors(true);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      toolboxUpdater = createToolboxUpdater(ghost);
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             new FlatGroundEnvironment(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.addSecondaryRobot(ghost);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.getRootRegistry().addChild(toolboxMainRegistry);
      simulationTestHelper.getSimulationConstructionSet().addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
      simulationTestHelper.start();

      //root.valkyrie.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.WalkingHighLevelHumanoidController.walkingCurrentState
      //root.valkyrie.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.toolboxMain.WalkingControllerPreviewToolboxController.WalkingHighLevelHumanoidController.walkingCurrentState
      YoEnum<WalkingStateEnum> controllerWalkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("walkingCurrentState");
      YoEnum<WalkingStateEnum> previewWalkingState = (YoEnum<WalkingStateEnum>) toolboxUpdater.getYoRegistry().findVariable("walkingCurrentState");

      assertNotNull(controllerWalkingState);
      assertNotNull(previewWalkingState);

      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      FullHumanoidRobotModel previewFullRobotModel = toolboxController.getFullRobotModel();
      simulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().addUpdatable(t -> toolboxUpdater.doControl());
      simulationTestHelper.addRobotControllerOnControllerThread(new Synchronizer(controllerWalkingState, previewWalkingState));

      SideDependentList<RigidBodyTrackingWatcher> footTrackingWatchers = new SideDependentList<>();
      SideDependentList<RigidBodyTrackingWatcher> handTrackingWatchers = new SideDependentList<>();
      RigidBodyTrackingWatcher headTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getHead(), previewFullRobotModel.getHead());
      RigidBodyTrackingWatcher chestTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getChest(), previewFullRobotModel.getChest());
      RigidBodyTrackingWatcher pelvisTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getPelvis(), previewFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTrackingWatcher footTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getFoot(robotSide),
                                                                                     previewFullRobotModel.getFoot(robotSide));
         simulationTestHelper.addRobotControllerOnControllerThread(footTrackingWatcher);
         footTrackingWatchers.put(robotSide, footTrackingWatcher);

         RigidBodyTrackingWatcher handTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getHand(robotSide),
                                                                                     previewFullRobotModel.getHand(robotSide));
         simulationTestHelper.addRobotControllerOnControllerThread(handTrackingWatcher);
         handTrackingWatchers.put(robotSide, handTrackingWatcher);
      }

      boolean success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(controllerFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      SideDependentList<Pose3DReadOnly> footPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(controllerFullRobotModel.getSoleFrame(robotSide));
         footPose.changeFrame(worldFrame);
         footPoses.put(robotSide, footPose);
      }

      WalkingControllerPreviewInputMessage input = new WalkingControllerPreviewInputMessage();
      Object<FootstepDataMessage> footstepDataList = input.getFootsteps().getFootstepDataList();
      int numberOfFootsteps = 10;

      for (int i = 0; i < numberOfFootsteps; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         Pose3DReadOnly footPose = footPoses.get(side);
         footstepDataList.add().set(HumanoidMessageTools.createFootstepDataMessage(side, footPose.getPosition(), footPose.getOrientation()));
      }

      simulationTestHelper.publishToController(new FootstepDataListMessage(input.getFootsteps()));
      toolboxInputManager.submitMessage(input);

      enableToolboxUpdater.set(true);

      while (!toolboxController.isWalkingControllerResetDone())
         toolboxUpdater.doControl();

      runToolboxController(50000);

      assertTrackingErrorMeanIsLow(headTrackingWatcher, 0.01, 0.015, 0.06, 0.20);
      assertTrackingErrorMeanIsLow(chestTrackingWatcher, 0.01, 0.015, 0.06, 0.20);
      assertTrackingErrorMeanIsLow(pelvisTrackingWatcher, 0.01, 0.015, 0.06, 0.20);

      for (RobotSide robotSide : RobotSide.values)
      {
         assertTrackingErrorMeanIsLow(footTrackingWatchers.get(robotSide), 0.01, 0.015, 0.06, 0.20);
         assertTrackingErrorMeanIsLow(handTrackingWatchers.get(robotSide), 0.04, 0.10, 0.06, 0.20); // I wonder if the tracking is off because the control is in joint-space.
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * The idea of this test is as follows:
    * <ol>
    * <li>Run the actual controller, make it go to a random robot configuration using its API so you
    * know it is reachable.
    * <li>Pause sim.
    * <li>Starts the preview and snap it to the current controller configuration.
    * <li>Resume simulation and preview at the same rate as the controller.
    * <li>Verify that the preview matches the controller over time
    * </ol>
    */
   @SuppressWarnings("unchecked")
   @Test
   public void testResetFeature()
   { // We check that the preview properly snaps to the current robot configuration before starting the preview.
      Random random = new Random(4720615);

      setup(getRobotModel().getControllerDT());
      createGhostRobot();
      simulationTestingParameters.setRunMultiThreaded(false);
      simulationTestingParameters.setUsePefectSensors(true);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      toolboxUpdater = createToolboxUpdater(ghost);
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             new FlatGroundEnvironment(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.addSecondaryRobot(ghost);
      // Spawning at a random location
      simulationTestHelperFactory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(EuclidCoreRandomTools.nextDouble(random, 5.0),
                                                                                              EuclidCoreRandomTools.nextDouble(random, 5.0),
                                                                                              0.0,
                                                                                              EuclidCoreRandomTools.nextDouble(random, Math.PI)));
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.getRootRegistry().addChild(toolboxMainRegistry);
      simulationTestHelper.getSimulationConstructionSet().addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
      simulationTestHelper.start();

      //root.valkyrie.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.WalkingHighLevelHumanoidController.walkingCurrentState
      //root.valkyrie.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.toolboxMain.WalkingControllerPreviewToolboxController.WalkingHighLevelHumanoidController.walkingCurrentState
      YoEnum<WalkingStateEnum> controllerWalkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("walkingCurrentState");
      YoEnum<WalkingStateEnum> previewWalkingState = (YoEnum<WalkingStateEnum>) toolboxUpdater.getYoRegistry().findVariable("walkingCurrentState");

      assertNotNull(controllerWalkingState);
      assertNotNull(previewWalkingState);

      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      FullHumanoidRobotModel previewFullRobotModel = toolboxController.getFullRobotModel();
      simulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().addUpdatable(t -> toolboxUpdater.doControl());
      Synchronizer synchronizer = new Synchronizer(controllerWalkingState, previewWalkingState);
      synchronizer.synchronize.set(false);
      simulationTestHelper.addRobotControllerOnControllerThread(synchronizer);

      SideDependentList<RigidBodyTrackingWatcher> footTrackingWatchers = new SideDependentList<>();
      SideDependentList<RigidBodyTrackingWatcher> handTrackingWatchers = new SideDependentList<>();
      RigidBodyTrackingWatcher headTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getHead(), previewFullRobotModel.getHead());
      RigidBodyTrackingWatcher chestTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getChest(), previewFullRobotModel.getChest());
      RigidBodyTrackingWatcher pelvisTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getPelvis(), previewFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTrackingWatcher footTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getFoot(robotSide),
                                                                                     previewFullRobotModel.getFoot(robotSide));
         simulationTestHelper.addRobotControllerOnControllerThread(footTrackingWatcher);
         footTrackingWatchers.put(robotSide, footTrackingWatcher);

         RigidBodyTrackingWatcher handTrackingWatcher = new RigidBodyTrackingWatcher(controllerFullRobotModel.getHand(robotSide),
                                                                                     previewFullRobotModel.getHand(robotSide));
         simulationTestHelper.addRobotControllerOnControllerThread(handTrackingWatcher);
         handTrackingWatchers.put(robotSide, handTrackingWatcher);
      }

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      RigidBodyBasics chest = controllerFullRobotModel.getChest();
      RigidBodyBasics pelvis = controllerFullRobotModel.getPelvis();
      MovingReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////// That's where we mess up with the robot configuration by requesting the controller to move around:

      { // Take a step so the feet are not square up
         FramePose3D footstep = new FramePose3D(controllerFullRobotModel.getSoleFrame(RobotSide.LEFT));
         footstep.changeFrame(worldFrame);
         footstep.prependTranslation(0.05, -0.05, 0.0);
         footstep.appendYawRotation(Math.toRadians(30.0));
         FootstepDataListMessage message = new FootstepDataListMessage();
         message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, footstep));
         simulationTestHelper.publishToController(message);
      }

      success = simulationTestHelper.simulateNow(3.0);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      { // Let's move the arm joints
         RigidBodyBasics hand = controllerFullRobotModel.getHand(robotSide);
         double[] desiredJointPositions = Stream.of(MultiBodySystemTools.createOneDoFJointPath(chest, hand))
                                                .mapToDouble(joint -> nextJointConfiguration(random, 0.5, joint)).toArray();
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 1.0, desiredJointPositions);
         simulationTestHelper.publishToController(message);
      }

      { // Let's move the neck joints
         RigidBodyBasics head = controllerFullRobotModel.getHead();
         double[] desiredJointPositions = Stream.of(MultiBodySystemTools.createOneDoFJointPath(chest, head))
                                                .mapToDouble(joint -> nextJointConfiguration(random, 0.5, joint)).toArray();
         NeckTrajectoryMessage message = HumanoidMessageTools.createNeckTrajectoryMessage(1.0, desiredJointPositions);
         simulationTestHelper.publishToController(message);
      }

      { // Now the chest
         OneDoFJointBasics[] spineJointsCopy = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);
         Arrays.asList(spineJointsCopy).forEach(joint -> joint.setQ(nextJointConfiguration(random, 0.5, joint)));
         spineJointsCopy[0].getPredecessor().updateFramesRecursively();

         FrameQuaternion chestOrientation = new FrameQuaternion(spineJointsCopy[spineJointsCopy.length - 1].getSuccessor().getBodyFixedFrame());
         chestOrientation.changeFrame(pelvisFrame);
         simulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(1.0, chestOrientation, pelvisFrame));
      }

      { // The pelvis
         FramePose3D pelvisPose = new FramePose3D(pelvisFrame);
         pelvisPose.changeFrame(worldFrame);
         pelvisPose.appendRotation(EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(10.0)));
         pelvisPose.prependTranslation(EuclidCoreRandomTools.nextPoint3D(random, 0.15));
         simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(1.0, pelvisPose));
      }

      success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(controllerFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      SideDependentList<Pose3DReadOnly> footPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(controllerFullRobotModel.getSoleFrame(robotSide));
         footPose.changeFrame(worldFrame);
         footPoses.put(robotSide, footPose);
      }

      WalkingControllerPreviewInputMessage input = new WalkingControllerPreviewInputMessage();
      Object<FootstepDataMessage> footstepDataList = input.getFootsteps().getFootstepDataList();
      int numberOfFootsteps = 2;

      for (int i = 0; i < numberOfFootsteps; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         Pose3D footPose = new Pose3D(footPoses.get(side));
         footPose.prependTranslation(0.10, 0.0, -0.01); // Lowering the footstep to trigger touchdown earlier on the controller side.
         footstepDataList.add().set(HumanoidMessageTools.createFootstepDataMessage(side, footPose));
      }

      simulationTestHelper.publishToController(new FootstepDataListMessage(input.getFootsteps()));
      toolboxInputManager.submitMessage(input);

      enableToolboxUpdater.set(true);

      while (!toolboxController.isWalkingControllerResetDone())
         toolboxUpdater.doControl();

      runToolboxController(1);

      synchronizer.synchronize.set(true);

      runToolboxController(50000);

      assertTrackingErrorMeanIsLow(headTrackingWatcher, 0.01, 0.015, 0.06, 0.20);
      assertTrackingErrorMeanIsLow(chestTrackingWatcher, 0.01, 0.015, 0.06, 0.20);
      assertTrackingErrorMeanIsLow(pelvisTrackingWatcher, 0.01, 0.015, 0.06, 0.20);

      for (RobotSide robotSide : RobotSide.values)
      {
         assertTrackingErrorMeanIsLow(footTrackingWatchers.get(robotSide), 0.01, 0.04, 0.06, 0.20);
         assertTrackingErrorMeanIsLow(handTrackingWatchers.get(robotSide), 0.05, 0.3, 0.1, 0.15); // I wonder if the tracking is off because the control is in joint-space.
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public static double nextJointConfiguration(Random random, double percentOfMotionRangeAllowed, OneDoFJointReadOnly joint)
   {
      double jointLimitLower = joint.getJointLimitLower();
      if (Double.isInfinite(jointLimitLower))
         jointLimitLower = -Math.PI;
      double jointLimitUpper = joint.getJointLimitUpper();
      if (Double.isInfinite(jointLimitUpper))
         jointLimitUpper = -Math.PI;
      double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
      jointLimitLower += 0.5 * rangeReduction;
      jointLimitUpper -= 0.5 * rangeReduction;
      return RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper);
   }

   private void assertTrackingErrorMeanIsLow(RigidBodyTrackingWatcher watcher,
                                             double positionTreshold,
                                             double orientationTreshold,
                                             double linearVelocityTreshold,
                                             double angularVelocityTreshold)
   {
      double positionMean = watcher.yoPositionTrackingErrorMean.getValue();
      double orientationMean = watcher.yoOrientationTrackingErrorMean.getValue();
      double linearVelocityMean = watcher.yoLinearVelocityTrackingErrorMean.getValue();
      double angularVelocityMean = watcher.yoAngularVelocityTrackingErrorMean.getValue();

      String errorMessage = "Poor tracking of the " + watcher.controllerBody.getName()
            + EuclidCoreIOTools.getStringOf("(",
                                            ")",
                                            ",",
                                            EuclidCoreIOTools.DEFAULT_FORMAT,
                                            positionMean,
                                            orientationMean,
                                            linearVelocityMean,
                                            angularVelocityMean);
      assertTrue(positionMean < positionTreshold, errorMessage);
      assertTrue(orientationMean < orientationTreshold, errorMessage);
      assertTrue(linearVelocityMean < linearVelocityTreshold, errorMessage);
      assertTrue(angularVelocityMean < angularVelocityTreshold, errorMessage);
   }

   private void runToolboxController(int maxNumberOfIterations)
   {
      enableToolboxUpdater.set(true);

      for (int i = 0; i < maxNumberOfIterations; i++)
      {
         if (!simulationTestHelper.simulateNow(0.25))
            break;
         if (toolboxController.isDone())
            break;
      }

      enableToolboxUpdater.set(false);
   }

   private Controller createToolboxUpdater(Robot robotToUpdate)
   {
      return new Controller()
      {
         private final JointReadOnly[] desiredJoints = MultiBodySystemTools.collectSubtreeJoints(toolboxController.getFullRobotModel().getElevator());
         private final ControllerOutputBasics scsInput = robotToUpdate.getControllerOutput();

         @Override
         public void doControl()
         {
            if (enableToolboxUpdater.getValue())
            {
               if (!initializationSucceeded.getBooleanValue())
                  initializationSucceeded.set(toolboxController.initialize());

               if (initializationSucceeded.getBooleanValue() && !pauseToolboxUpdater.getValue())
               {
                  toolboxController.updateInternal();

                  for (JointReadOnly joint : desiredJoints)
                  {
                     scsInput.getJointOutput(joint).setConfiguration(joint);
                  }
               }
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return toolboxMainRegistry;
         }

         @Override
         public String getName()
         {
            return toolboxMainRegistry.getName();
         }
      };
   }

   private class RigidBodyTrackingWatcher implements RobotController
   {
      private final YoRegistry registry;
      private final RigidBodyReadOnly controllerBody;
      private final RigidBodyReadOnly previewBody;

      private final YoDouble positionTrackingErrorMagnitude;
      private final YoDouble orientationTrackingErrorMagnitude;
      private final YoDouble linearVelocityTrackingErrorMagnitude;
      private final YoDouble angularVelocityTrackingErrorMagnitude;

      private final Mean positionTrackingErrorMean = new Mean();
      private final Mean orientationTrackingErrorMean = new Mean();
      private final Mean linearVelocityTrackingErrorMean = new Mean();
      private final Mean angularVelocityTrackingErrorMean = new Mean();

      private final YoDouble yoPositionTrackingErrorMean;
      private final YoDouble yoOrientationTrackingErrorMean;
      private final YoDouble yoLinearVelocityTrackingErrorMean;
      private final YoDouble yoAngularVelocityTrackingErrorMean;

      public RigidBodyTrackingWatcher(RigidBodyReadOnly controllerBody, RigidBodyReadOnly previewBody)
      {
         this.controllerBody = controllerBody;
         this.previewBody = previewBody;

         registry = new YoRegistry(controllerBody.getName() + "TrackingWatcher");

         positionTrackingErrorMagnitude = new YoDouble(controllerBody.getName() + "PositionTrackingErrorMagnitude", registry);
         orientationTrackingErrorMagnitude = new YoDouble(controllerBody.getName() + "OrientationTrackingErrorMagnitude", registry);
         linearVelocityTrackingErrorMagnitude = new YoDouble(controllerBody.getName() + "LinearVelocityTrackingErrorMagnitude", registry);
         angularVelocityTrackingErrorMagnitude = new YoDouble(controllerBody.getName() + "AngularVelocityTrackingErrorMagnitude", registry);

         yoPositionTrackingErrorMean = new YoDouble(controllerBody.getName() + "PositionTrackingErrorMean", registry);
         yoOrientationTrackingErrorMean = new YoDouble(controllerBody.getName() + "OrientationTrackingErrorMean", registry);
         yoLinearVelocityTrackingErrorMean = new YoDouble(controllerBody.getName() + "LinearVelocityTrackingErrorMean", registry);
         yoAngularVelocityTrackingErrorMean = new YoDouble(controllerBody.getName() + "AngularVelocityTrackingErrorMean", registry);
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public void doControl()
      {
         if (!enableToolboxUpdater.getValue())
         {
            positionTrackingErrorMagnitude.set(0.0);
            orientationTrackingErrorMagnitude.set(0.0);
            linearVelocityTrackingErrorMagnitude.set(0.0);
            angularVelocityTrackingErrorMagnitude.set(0.0);
            return;
         }

         MovingReferenceFrame controllerBodyFixedFrame = controllerBody.getBodyFixedFrame();
         MovingReferenceFrame previewBodyFixedFrame = previewBody.getBodyFixedFrame();

         FramePose3D controllerBodyPose = new FramePose3D(controllerBodyFixedFrame);
         controllerBodyPose.changeFrame(previewBodyFixedFrame);

         positionTrackingErrorMagnitude.set(controllerBodyPose.getPosition().distanceFromOrigin());
         orientationTrackingErrorMagnitude.set(EuclidCoreTools.trimAngleMinusPiToPi(controllerBodyPose.getOrientation().getAngle()));

         SpatialVector controllerVelocity = new SpatialVector(controllerBodyFixedFrame.getTwistOfFrame());
         controllerVelocity.changeFrame(worldFrame);
         SpatialVector previewVelocity = new SpatialVector(previewBodyFixedFrame.getTwistOfFrame());
         previewVelocity.changeFrame(worldFrame);
         SpatialVector deltaVelocity = new SpatialVector(controllerVelocity);
         deltaVelocity.sub(previewVelocity);

         linearVelocityTrackingErrorMagnitude.set(deltaVelocity.getLinearPart().length());
         angularVelocityTrackingErrorMagnitude.set(deltaVelocity.getAngularPart().length());

         positionTrackingErrorMean.increment(positionTrackingErrorMagnitude.getValue());
         orientationTrackingErrorMean.increment(orientationTrackingErrorMagnitude.getValue());
         linearVelocityTrackingErrorMean.increment(linearVelocityTrackingErrorMagnitude.getValue());
         angularVelocityTrackingErrorMean.increment(angularVelocityTrackingErrorMagnitude.getValue());

         yoPositionTrackingErrorMean.set(positionTrackingErrorMean.getResult());
         yoOrientationTrackingErrorMean.set(orientationTrackingErrorMean.getResult());
         yoLinearVelocityTrackingErrorMean.set(linearVelocityTrackingErrorMean.getResult());
         yoAngularVelocityTrackingErrorMean.set(angularVelocityTrackingErrorMean.getResult());
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return registry.getName();
      }
   }

   /**
    * This controller's goal is to re-sync the preview to the walking controller. When the preview is
    * late at a state change, it is ran multiple times until it catches up. When the preview is ahead
    * at a state change, it is paused until the controller catches up.
    */
   private class Synchronizer implements RobotController
   {
      private final YoRegistry registry = new YoRegistry(getName());

      private final YoBoolean synchronize = new YoBoolean("synchronize", registry);
      private final YoBoolean isControllerLeadingPreview = new YoBoolean("isControllerLeadingPreview", registry);
      private final YoBoolean isPreviewLeadingController = new YoBoolean("isPreviewLeadingController", registry);
      private final YoInteger controllerStateChangeCount = new YoInteger("controllerStateChangeCount", registry);
      private final YoInteger previewStateChangeCount = new YoInteger("previewStateChangeCount", registry);

      public Synchronizer(YoEnum<WalkingStateEnum> controllerState, YoEnum<WalkingStateEnum> previewState)
      {
         synchronize.set(true);
         controllerState.addListener(v -> controllerStateChangeCount.increment());
         previewState.addListener(v -> previewStateChangeCount.increment());
         controllerStateChangeCount.addListener(v -> processStateChanges());
         previewStateChangeCount.addListener(v -> processStateChanges());
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public void doControl()
      {
         if (!synchronize.getValue())
         {
            controllerStateChangeCount.set(0, false);
            previewStateChangeCount.set(0, false);
            isControllerLeadingPreview.set(false);
            isPreviewLeadingController.set(false);
            return;
         }

         if (!enableToolboxUpdater.getValue())
            return;

         while (isControllerLeadingPreview.getValue())
         {
            toolboxUpdater.doControl();
         }

         if (isPreviewLeadingController.getValue())
            pauseToolboxUpdater.set(true);
         else
            pauseToolboxUpdater.set(false);
      }

      private void processStateChanges()
      {
         if (!synchronize.getValue())
         {
            controllerStateChangeCount.set(0, false);
            previewStateChangeCount.set(0, false);
            isControllerLeadingPreview.set(false);
            isPreviewLeadingController.set(false);
            return;
         }

         if (controllerStateChangeCount.getValue() > previewStateChangeCount.getValue())
         {
            isControllerLeadingPreview.set(true);
            isPreviewLeadingController.set(false);
         }
         else if (controllerStateChangeCount.getValue() == previewStateChangeCount.getValue())
         {
            isControllerLeadingPreview.set(false);
            isPreviewLeadingController.set(false);
         }
         else
         {
            isControllerLeadingPreview.set(false);
            isPreviewLeadingController.set(true);
         }
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }

   private FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(double initialYaw)
   {
      FullHumanoidRobotModel initialFullRobotModel = getRobotModel().createFullRobotModel();
      getRobotModel().getDefaultRobotInitialSetup(0.0, initialYaw).initializeRobot(initialFullRobotModel.getElevator());
      initialFullRobotModel.updateFrames();
      return initialFullRobotModel;
   }
}
