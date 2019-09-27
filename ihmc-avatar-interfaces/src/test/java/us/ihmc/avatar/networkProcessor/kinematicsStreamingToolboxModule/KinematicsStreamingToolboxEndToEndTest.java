package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Color;
import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.AfterEach;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageReplay;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class KinematicsStreamingToolboxEndToEndTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   public static final double toolboxControllerPeriod = 5.0e-3;

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private SimulationConstructionSet scs;
   private KinematicsStreamingToolboxMessageReplay kinematicsStreamingToolboxMessageReplay;

   protected CommandInputManager commandInputManager;
   protected StatusMessageOutputManager statusOutputManager;
   protected YoVariableRegistry toolboxRegistry;
   protected YoGraphicsListRegistry yoGraphicsListRegistry;
   protected FullHumanoidRobotModel desiredFullRobotModel;
   protected KinematicsStreamingToolboxController toolboxController;
   private HumanoidFloatingRootJointRobot toolboxGhost;
   private RealtimeRos2Node toolboxRos2Node;
   protected static final YoAppearanceRGBColor toolboxGhostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);

   public abstract DRCRobotModel newRobotModel();

   protected void runTest(InputStream inputStream) throws IOException, SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = newRobotModel();
      String robotName = robotModel.getSimpleRobotName();
      kinematicsStreamingToolboxMessageReplay = new KinematicsStreamingToolboxMessageReplay(robotName,
                                                                                                                                    inputStream,
                                                                                                                                    PubSubImplementation.INTRAPROCESS);

      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         simulationTestingParameters.setKeepSCSUp(true);

      DRCRobotModel toolboxGhostRobotModel = newRobotModel();
      toolboxGhost = KinematicsStreamingToolboxControllerTest.createSCSRobot(toolboxGhostRobotModel, "ghost", toolboxGhostApperance);
      KinematicsStreamingToolboxControllerTest.hideRobot(toolboxGhost);
      toolboxGhost.setDynamic(false);
      toolboxGhost.setGravity(0);

      RobotConfigurationData initialRobotConfigurationData = kinematicsStreamingToolboxMessageReplay.getInitialConfiguration();
      OffsetAndYawRobotInitialSetup initialSimulationSetup = new OffsetAndYawRobotInitialSetup(initialRobotConfigurationData.getRootTranslation().getX(),
                                                                                               initialRobotConfigurationData.getRootTranslation().getY(),
                                                                                               0.0,
                                                                                               initialRobotConfigurationData.getRootOrientation().getYaw());

      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      environment.addEnvironmentRobot(toolboxGhost);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setStartingLocation(initialSimulationSetup);

      toolboxRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "toolbox_node");
      createToolboxController(toolboxGhostRobotModel);
      toolboxGhost.setController(new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(toolboxGhost, desiredFullRobotModel);

         @Override
         public void initialize()
         {
         }

         @Override
         public void doControl()
         {
            toolboxController.update();

            jointAnglesWriter.setWriteJointVelocities(false);
            jointAnglesWriter.setWriteJointAccelerations(false);
            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return toolboxRegistry;
         }
      }, (int) (toolboxControllerPeriod / toolboxGhostRobotModel.getSimulateDT()));

      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      Point3D cameraFix = new Point3D(initialRobotConfigurationData.getRootTranslation());
      Point3D cameraPosition = new Point3D(cameraFix);
      cameraPosition.add(-7.0, -9.0, 4.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      scs = drcSimulationTestHelper.getSimulationConstructionSet();

      humanoidReferenceFrames.updateFrames();

      kinematicsStreamingToolboxMessageReplay.initialize(scs.getTime());

      AtomicBoolean doneWithReplay = new AtomicBoolean(false);
      scs.addScript(time -> doneWithReplay.set(!kinematicsStreamingToolboxMessageReplay.update(time)));
      scs.setSimulateDoneCriterion(doneWithReplay::get);
      scs.simulate();

      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();
   }

   public void createToolboxController(DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator toolboxSubGenerator = KinematicsStreamingToolboxModule.getSubscriberTopicNameGenerator(robotName);
      MessageTopicNameGenerator toolboxPubGenerator = KinematicsStreamingToolboxModule.getPublisherTopicNameGenerator(robotName);

      desiredFullRobotModel = robotModel.createFullRobotModel();
      toolboxRegistry = new YoVariableRegistry("toolboxMain");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsStreamingToolboxCommandConverter(desiredFullRobotModel));
      statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      new ControllerNetworkSubscriber(toolboxSubGenerator, commandInputManager, toolboxPubGenerator, statusOutputManager, toolboxRos2Node);

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   robotModel.getControllerDT(),
                                                                   toolboxControllerPeriod,
                                                                   yoGraphicsListRegistry,
                                                                   toolboxRegistry);
      toolboxController.setOutputPublisher(drcSimulationTestHelper::publishToController);

      ROS2Tools.createCallbackSubscription(toolboxRos2Node,
                                           RobotConfigurationData.class,
                                           controllerPubGenerator,
                                           s -> toolboxController.updateRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(toolboxRos2Node,
                                           CapturabilityBasedStatus.class,
                                           controllerPubGenerator,
                                           s -> toolboxController.updateCapturabilityBasedStatus(s.takeNextData()));
      toolboxRos2Node.spin();
   }

   @AfterEach
   public void tearDown()
   {
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }
      fullRobotModel = null;
      humanoidReferenceFrames = null;
      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
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
      if (toolboxRos2Node != null)
      {
         toolboxRos2Node.destroy();
         toolboxRos2Node = null;
      }
   }
}
