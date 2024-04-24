package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotConfigurationDataInitialSetup;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageReplay;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerOutputBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.io.InputStream;

import static org.junit.jupiter.api.Assertions.*;

@Tag("humanoid-toolbox")
public abstract class KinematicsStreamingToolboxEndToEndTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   public static final double toolboxControllerPeriod = 5.0e-3;

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private FullHumanoidRobotModel fullRobotModel;
   private CommonHumanoidReferenceFrames humanoidReferenceFrames;
   private KinematicsStreamingToolboxMessageReplay kinematicsStreamingToolboxMessageReplay;

   protected CommandInputManager commandInputManager;
   protected StatusMessageOutputManager statusOutputManager;
   protected YoRegistry toolboxRegistry;
   protected YoGraphicsListRegistry yoGraphicsListRegistry;
   protected FullHumanoidRobotModel desiredFullRobotModel;
   protected KinematicsStreamingToolboxController toolboxController;
   private Robot toolboxGhost;
   private RealtimeROS2Node toolboxROS2Node;
   protected static final MaterialDefinition toolboxGhostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));

   public abstract DRCRobotModel newRobotModel();

   protected void runTest(InputStream inputStream) throws IOException
   {
      DRCRobotModel robotModel = newRobotModel();
      String robotName = robotModel.getSimpleRobotName();
      kinematicsStreamingToolboxMessageReplay = new KinematicsStreamingToolboxMessageReplay(robotName, inputStream, PubSubImplementation.INTRAPROCESS);

      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         simulationTestingParameters.setKeepSCSUp(true);

      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      ghostRobotDefinition.setName("ghost");
      ghostRobotDefinition.ignoreAllJoints();
      RobotDefinitionTools.setRobotDefinitionMaterial(ghostRobotDefinition, toolboxGhostMaterial);
      toolboxGhost = new Robot(ghostRobotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      KinematicsStreamingToolboxControllerTest.hideRobot(toolboxGhost);

      RobotConfigurationData initialRobotConfigurationData = kinematicsStreamingToolboxMessageReplay.getInitialConfiguration();

      FlatGroundEnvironment environment = new FlatGroundEnvironment(getGroundHeight(initialRobotConfigurationData));
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setRobotInitialSetup(new RobotConfigurationDataInitialSetup(initialRobotConfigurationData,
                                                                                              newRobotModel().createFullRobotModel()));
      simulationTestHelperFactory.addSecondaryRobot(toolboxGhost);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();

      toolboxROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "toolbox_node");
      createToolboxController(robotModel);
      toolboxGhost.addThrottledController(new Controller()
      {
         private final JointReadOnly[] desiredJoints = MultiBodySystemTools.collectSubtreeJoints(desiredFullRobotModel.getElevator());
         private final ControllerOutputBasics scsInput = toolboxGhost.getControllerOutput();

         @Override
         public void doControl()
         {
            toolboxController.update();

            for (JointReadOnly joint : desiredJoints)
            {
               scsInput.getJointOutput(joint).setConfiguration(joint);
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return toolboxRegistry;
         }
      }, toolboxControllerPeriod);

      simulationTestHelper.start();

      Point3D cameraFix = new Point3D(initialRobotConfigurationData.getRootPosition());
      Point3D cameraPosition = new Point3D(cameraFix);
      cameraPosition.add(-7.0, -9.0, 4.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);

      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(0.5));

      fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();

      humanoidReferenceFrames.updateFrames();

      kinematicsStreamingToolboxMessageReplay.initialize(simulationTestHelper.getSimulationTime());

      simulationTestHelper.addSimulationTerminalCondition(() -> !kinematicsStreamingToolboxMessageReplay.update(simulationTestHelper.getSimulationTime()));
      simulationTestHelper.simulateNow(1000.0);
   }

   private double getGroundHeight(RobotConfigurationData initialRobotConfigurationData)
   {
      FullHumanoidRobotModel fullRobotModel = newRobotModel().createFullRobotModel();
      ;
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotConfigurationData = new RobotConfigurationDataInitialSetup(initialRobotConfigurationData,
                                                                                                                        fullRobotModel);

      robotConfigurationData.initializeFullRobotModel(fullRobotModel);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      FramePoint3D footHeight = new FramePoint3D(referenceFrames.getMidFeetUnderPelvisFrame());
      footHeight.changeFrame(ReferenceFrame.getWorldFrame());
      return footHeight.getZ();
   }

   protected static void hideRobot(HumanoidFloatingRootJointRobot robot)
   {
      robot.setPositionInWorld(new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
   }

   public void createToolboxController(DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();
      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ROS2Topic<?> toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      ROS2Topic<?> toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

      desiredFullRobotModel = robotModel.createFullRobotModel();
      toolboxRegistry = new YoRegistry("toolboxMain");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      new ControllerNetworkSubscriber(toolboxInputTopic, commandInputManager, toolboxOutputTopic, statusOutputManager, toolboxROS2Node);

      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      parameters.setToolboxUpdatePeriod(toolboxControllerPeriod);

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   parameters,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   yoGraphicsListRegistry,
                                                                   toolboxRegistry);
      toolboxController.setTrajectoryMessagePublisher(simulationTestHelper::publishToController);
      toolboxController.setStreamingMessagePublisher(simulationTestHelper::publishToController);

      RobotConfigurationDataBasedUpdater robotStateUpdater = new RobotConfigurationDataBasedUpdater();
      toolboxController.setRobotStateUpdater(robotStateUpdater);
      toolboxROS2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class),
                                         s ->robotStateUpdater.setRobotConfigurationData(s.takeNextData()));

      toolboxROS2Node.createSubscription(controllerOutputTopic.withTypeName(CapturabilityBasedStatus.class),
                                         s -> toolboxController.updateCapturabilityBasedStatus(s.takeNextData()));
      toolboxROS2Node.spin();
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }
      fullRobotModel = null;
      humanoidReferenceFrames = null;
      if (kinematicsStreamingToolboxMessageReplay != null)
      {
         kinematicsStreamingToolboxMessageReplay.close();
         kinematicsStreamingToolboxMessageReplay = null;
      }

      commandInputManager = null;
      statusOutputManager = null;
      toolboxRegistry = null;
      yoGraphicsListRegistry = null;
      desiredFullRobotModel = null;
      toolboxController = null;
      toolboxGhost = null;
      if (toolboxROS2Node != null)
      {
         toolboxROS2Node.destroy();
         toolboxROS2Node = null;
      }
   }

   protected static HumanoidFloatingRootJointRobot createSCSRobot(DRCRobotModel ghostRobotModel, String robotName, MaterialDefinition robotMaterial)
   {
      RobotDefinition robotDefinition = ghostRobotModel.getRobotDefinition();
      robotDefinition.setName(robotName);
      RobotDefinitionTools.setRobotDefinitionMaterial(robotDefinition, robotMaterial);
      HumanoidFloatingRootJointRobot scsRobot = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      scsRobot.getRootJoint().setPinned(true);
      scsRobot.setDynamic(false);
      scsRobot.setGravity(0);
      return scsRobot;
   }
}
