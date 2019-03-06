package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

@Tag("humanoid-behaviors")
public class AtlasPatrolBehaviorTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private AtlasBehaviorTestYoVariables variables;
   private GoalOrientedTestConductor conductor;
   private AvatarSimulation avatarSimulation;
   private AtlasRobotModel robotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;

   @Test
   public void testDoNothingBehavior()
   {
      // do nothing!!!
   }

   @Test
   public void testStepInPlaceBehavior()
   {
      FootstepDataListMessage footMessage = new FootstepDataListMessage();
      MovingReferenceFrame stepFrame = avatarSimulation.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT);
      FramePoint3D footLocation = new FramePoint3D(stepFrame);
      FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
      footLocation.changeFrame(ReferenceFrame.getWorldFrame());
      footOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      footMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, footLocation, footOrientation));
      footMessage.setAreFootstepsAdjustable(true);

      footstepDataListPublisher.publish(footMessage);

      AtlasTestScripts.nextTouchdown(conductor, variables, 3.0);
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      conductor.concludeTesting();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      SimulationConstructionSet scs = createHumanoidSimulation(new FlatGroundEnvironment());
      variables = new AtlasBehaviorTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AtlasTestScripts.standUp(conductor, variables);
   }

   private SimulationConstructionSet createHumanoidSimulation(CommonAvatarEnvironmentInterface environment)
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setTimePerRecordTick(robotModel.getControllerDT());

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
      {
         contactableBodiesFactory.addAdditionalContactPoint(contactPointParameters.getAdditionalContactRigidBodyNames().get(i),
                                                            contactPointParameters.getAdditionalContactNames().get(i),
                                                            contactPointParameters.getAdditionalContactTransforms().get(i));
      }

      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "humanoid_simulation_controller");

      HighLevelHumanoidControllerFactory controllerFactory =
            new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                   robotModel.getSensorInformation().getFeetForceSensorNames(),
                                                   robotModel.getSensorInformation().getFeetContactSensorNames(),
                                                   robotModel.getSensorInformation().getWristForceSensorNames(),
                                                   robotModel.getHighLevelControllerParameters(),
                                                   robotModel.getWalkingControllerParameters(),
                                                   robotModel.getCapturePointPlannerParameters());
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, DO_NOTHING_BEHAVIOR);
      controllerFactory.addControllerFailureTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.setInitialState(HighLevelControllerName.WALKING);
      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeRos2Node);

      AvatarSimulationFactory avatarSimulationFactory = new AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setShapeCollisionSettings(robotModel.getShapeCollisionSettings());
      avatarSimulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvironment(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotModel.getDefaultRobotInitialSetup(0.0, 0.0));
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setRealtimeRos2Node(realtimeRos2Node);
      avatarSimulationFactory.setCreateYoVariableServer(true);

      avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      MessageTopicNameGenerator controllerTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.INTRAPROCESS, "humanoid_behavior_module");
      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(), controllerTopicNameGenerator);

      avatarSimulation.start();
      realtimeRos2Node.spin();  // TODO Should probably happen in start()

      return avatarSimulation.getSimulationConstructionSet();
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(AtlasPatrolBehaviorTest.class + " after class.");
   }
}
