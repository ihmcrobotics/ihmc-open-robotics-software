package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotConfigurationDataInitialSetup;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageReplay;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.assertTrue;

@Tag("humanoid-toolbox")
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
   protected YoRegistry toolboxRegistry;
   protected YoGraphicsListRegistry yoGraphicsListRegistry;
   protected FullHumanoidRobotModel desiredFullRobotModel;
   protected KinematicsStreamingToolboxController toolboxController;
   private HumanoidFloatingRootJointRobot toolboxGhost;
   private RealtimeROS2Node toolboxROS2Node;
   protected static final YoAppearanceRGBColor toolboxGhostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);

   public abstract DRCRobotModel newRobotModel();

   protected void runTest(InputStream inputStream) throws IOException, SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = newRobotModel();
      String robotName = robotModel.getSimpleRobotName();
      kinematicsStreamingToolboxMessageReplay = new KinematicsStreamingToolboxMessageReplay(robotName, inputStream, PubSubImplementation.INTRAPROCESS);

      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         simulationTestingParameters.setKeepSCSUp(true);

      DRCRobotModel toolboxGhostRobotModel = newRobotModel();
      toolboxGhost = KinematicsStreamingToolboxControllerTest.createSCSRobot(toolboxGhostRobotModel, "ghost", toolboxGhostApperance);
      KinematicsStreamingToolboxControllerTest.hideRobot(toolboxGhost);
      toolboxGhost.setDynamic(false);
      toolboxGhost.setGravity(0);

      RobotConfigurationData initialRobotConfigurationData = kinematicsStreamingToolboxMessageReplay.getInitialConfiguration();

      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      environment.addEnvironmentRobot(toolboxGhost);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setInitialSetup(new RobotConfigurationDataInitialSetup(initialRobotConfigurationData, newRobotModel().createFullRobotModel()));

      toolboxROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "toolbox_node");
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
         public YoRegistry getYoRegistry()
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
      ROS2Topic controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Topic toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      ROS2Topic toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

      desiredFullRobotModel = robotModel.createFullRobotModel();
      toolboxRegistry = new YoRegistry("toolboxMain");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());

      new ControllerNetworkSubscriber(toolboxInputTopic, commandInputManager, toolboxOutputTopic, statusOutputManager, toolboxROS2Node);

      toolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                   statusOutputManager,
                                                                   desiredFullRobotModel,
                                                                   robotModel,
                                                                   robotModel.getControllerDT(),
                                                                   toolboxControllerPeriod,
                                                                   yoGraphicsListRegistry,
                                                                   toolboxRegistry);
      toolboxController.setTrajectoryMessagePublisher(drcSimulationTestHelper::publishToController);
      toolboxController.setStreamingMessagePublisher(drcSimulationTestHelper::publishToController);

      ROS2Tools.createCallbackSubscriptionTypeNamed(toolboxROS2Node,
                                                    RobotConfigurationData.class,
                                                    controllerOutputTopic,
                                           s -> toolboxController.updateRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(toolboxROS2Node,
                                                    CapturabilityBasedStatus.class,
                                                    controllerOutputTopic,
                                           s -> toolboxController.updateCapturabilityBasedStatus(s.takeNextData()));
      toolboxROS2Node.spin();
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
      if (toolboxROS2Node != null)
      {
         toolboxROS2Node.destroy();
         toolboxROS2Node = null;
      }
   }
}
