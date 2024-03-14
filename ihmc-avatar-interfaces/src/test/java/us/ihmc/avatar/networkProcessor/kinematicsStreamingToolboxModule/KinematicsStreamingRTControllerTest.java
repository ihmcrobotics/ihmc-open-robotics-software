package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import javafx.application.Platform;
import javafx.scene.control.Button;
import org.junit.jupiter.api.AfterEach;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest.IKStreamingMessageGenerator;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest.IKStreamingTestRunParameters;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest.SimRunner;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters.ClockType;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.WalkingCommandConsumer;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerOutputBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import static org.junit.jupiter.api.Assertions.*;

public abstract class KinematicsStreamingRTControllerTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected static final MaterialDefinition ghostMaterial = new MaterialDefinition(ColorDefinitions.Yellow().derive(0, 1, 1, 0.25));
   protected static final boolean visualize = simulationTestingParameters.getCreateGUI();

   protected YoRegistry toolboxRegistry;
   protected SCS2AvatarTestingSimulation simulationTestHelper;
   private IHMCROS2Publisher<KinematicsStreamingToolboxInputMessage> inputPublisher;
   private IHMCROS2Publisher<ToolboxStateMessage> statePublisher;

   private Robot ghost;

   /**
    * Returns a <b>new</b> instance of the robot model that will be modified in this test to create
    * ghost robots.
    */
   public abstract DRCRobotModel newRobotModel();

   public void setupWithWalkingController(KinematicsStreamingToolboxParameters toolboxParameters)
   {
      DRCRobotModel robotModel = newRobotModel();

      String robotName = robotModel.getSimpleRobotName();

      RobotDefinition ghostDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      RobotDefinitionTools.setRobotDefinitionMaterial(ghostDefinition, ghostMaterial);
      ghostDefinition.ignoreAllJoints();
      ghostDefinition.setName("ghost");
      ghost = new Robot(ghostDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      KinematicsStreamingToolboxControllerTest.hideRobot(ghost);

      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             testEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.addSecondaryRobot(ghost);

      simulationTestHelperFactory.createIKStreamingRealTimeController(true);
      toolboxParameters.setClockType(ClockType.FIXED_DT);
      simulationTestHelperFactory.setIKStreamingParameters(toolboxParameters);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      YoBoolean isAutomaticManipulationAbortEnabled = (YoBoolean) simulationTestHelper.getControllerRegistry()
                                                                                      .findVariable(WalkingCommandConsumer.class.getSimpleName(),
                                                                                                    "isAutomaticManipulationAbortEnabled");
      isAutomaticManipulationAbortEnabled.set(false); // TODO This is a hack to prevent the walking controller from aborting the manipulation task.

      ROS2Node ros2Node = simulationTestHelper.getROS2Node();
      ROS2Topic<?> toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      ROS2Topic<?> toolboxOutputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

      inputPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, KinematicsStreamingToolboxInputMessage.class, toolboxInputTopic);
      statePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, toolboxInputTopic);

      // TODO Maybe add the ghost robot again?

      Controller toolboxUpdater = new Controller()
      {
         private final JointReadOnly[] desiredJoints = MultiBodySystemTools.collectSubtreeJoints(simulationTestHelper.getAvatarSimulation()
                                                                                                                     .getIKStreamingRTThread()
                                                                                                                     .getFullRobotModel()
                                                                                                                     .getElevator());
         private final ControllerOutputBasics scsInput = ghost.getControllerOutput();

         @Override
         public void doControl()
         {
            try
            {
               for (JointReadOnly joint : desiredJoints)
               {
                  scsInput.getJointOutput(joint).setConfiguration(joint);
               }
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return toolboxRegistry;
         }
      };
      ghost.addThrottledController(toolboxUpdater, toolboxParameters.getToolboxUpdatePeriod());
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
         simulationTestHelper = null;
      }

      toolboxRegistry = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testStreamingToController(IKStreamingTestRunParameters ikStreamingTestRunParameters)
   {
      setupWithWalkingController(ikStreamingTestRunParameters.toolboxParameters());
      //      SpyTrackingController spyController = new SpyTrackingController(toolboxRegistry,
      //                                                                      simulationTestHelper.getAvatarSimulation().getIKStreamingRTThread().getFullRobotModel(),
      //                                                                      simulationTestHelper.getControllerFullRobotModel());
      //      simulationTestHelper.addRobotControllerOnControllerThread(spyController);

      if (ikStreamingTestRunParameters.getRegistry() != null)
         simulationTestHelper.addRegistry(ikStreamingTestRunParameters.getRegistry());

      IKStreamingMessageGenerator ikStreamingMessageGenerator = ikStreamingTestRunParameters.messageGenerator();

      simulationTestHelper.getSimulationConstructionSet().getRobots().get(0).addThrottledController(new Controller()
      {
         @Override
         public void initialize()
         {
            try
            {
               ikStreamingMessageGenerator.initialize();
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }

         @Override
         public void doControl()
         {
            try
            {
               double time = simulationTestHelper.getSimulationTime();
               KinematicsStreamingToolboxInputMessage message = ikStreamingMessageGenerator.update(time);
               if (message != null)
                  inputPublisher.publish(message);
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      }, ikStreamingTestRunParameters.messageGeneratorDT());

      simulationTestHelper.start();
      SimRunner simRunner = new SimRunner(simulationTestHelper);

      SimulationConstructionSet2 scs = simulationTestHelper.getSimulationConstructionSet();
      scs.waitUntilVisualizerFullyUp();
      Platform.runLater(() ->
                        {
                           Button restart = new Button("Restart");
                           restart.setOnAction(event -> simRunner.reset());
                           scs.addCustomGUIControl(restart);
                        });

      assertTrue(simRunner.simulateNow(0.5));
      wakeupToolbox();

      assertTrue(simRunner.simulateNow(ikStreamingTestRunParameters.simulationDuration()));

      KinematicsStreamingToolboxInputMessage message = new KinematicsStreamingToolboxInputMessage();
      message.setStreamToController(false);
      inputPublisher.publish(message);

      //      // Asserts that the spy did run and that the toolbox or something did not just hang
      //      double handPositionMeanError = spyController.getHandPositionMeanError();
      //      double handOrientationMeanError = spyController.getHandOrientationMeanError();
      //      assertNotEquals(0.0, handPositionMeanError);
      //      assertNotEquals(0.0, handOrientationMeanError);
      //      // TODO Pretty bad assertions here, need to figure out how to improve this test later.
      //      System.out.println("Position error avg: " + handPositionMeanError + ", orientation error avg: " + handOrientationMeanError);
      //      assertTrue(handPositionMeanError < ikStreamingTestRunParameters.handPositionMeanErrorThreshold(), "Mean position error is: " + handPositionMeanError);
      //      assertTrue(handOrientationMeanError < ikStreamingTestRunParameters.handOrientationMeanErrorThreshold(),
      //                 "Mean orientation error is: " + handOrientationMeanError);
   }

   protected void wakeupToolbox()
   {
      ToolboxStateMessage wakeupMessage = new ToolboxStateMessage();
      wakeupMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      statePublisher.publish(wakeupMessage);
   }

   protected void sleepToolbox()
   {
      ToolboxStateMessage wakeupMessage = new ToolboxStateMessage();
      wakeupMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      statePublisher.publish(wakeupMessage);
   }
}
