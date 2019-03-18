package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.BehaviorBackpack;
import us.ihmc.humanoidBehaviors.BehaviorTeleop;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.humanoidBehaviors.tools.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidFrames;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.log.LogTools;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertTrue;

@Tag("humanoid-behaviors")
public class AtlasPatrolBehaviorTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private AtlasBehaviorTestYoVariables variables;
   private GoalOrientedTestConductor conductor;
   private AtlasRobotModel robotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private IHMCROS2Publisher<AbortWalkingMessage> abortPublisher;
   private Ros2Node ros2Node;

   @Test
   public void testPatrolBehavior() throws IOException
   {
      new MultiStageFootstepPlanningModule(robotModel, null, false, PubSubImplementation.INTRAPROCESS);

      SharedMemoryMessager messager = new SharedMemoryMessager(BehaviorBackpack.getBehaviorAPI());
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      LogTools.info("Creating behavior module");
      BehaviorBackpack.createForTest(robotModel, messager);

      LogTools.info("Creating behavior teleop");
      BehaviorTeleop behaviorTeleop = BehaviorTeleop.createForTest(robotModel, messager);
      behaviorTeleop.getModuleMessager().registerTopicListener(PatrolBehavior.API.CurrentState, state -> LogTools.info("Patrol state: {}", state));

      AtlasTestScripts.wait(conductor, variables, 0.25);  // allows to update frames

      ArrayList<Pose3D> waypoints = new ArrayList<Pose3D>();

      waypoints.add(new Pose3D(1.0, -0.2, 0.0, 20.0, 0.0, 0.0));
      waypoints.add(new Pose3D(1.0, -1.0, 0.0, 180.0, 0.0, 0.0));

      behaviorTeleop.setWaypoints(waypoints);

      behaviorTeleop.goToWaypoint(0);


//      FramePose3D currentGoalWaypoint = new FramePose3D();
//      currentGoalWaypoint.prependTranslation(2.0, 0.0, 0.0);

      AtlasTestScripts.wait(conductor, variables, 10000.0);  // allows to update frames

//      AtlasTestScripts.takeSteps(conductor, variables, output.getFootstepDataList().getFootstepDataList().size(), 6.0);
//
//      AtlasTestScripts.holdDoubleSupport(conductor, variables, 3.0, 6.0);
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
      SimulationConstructionSet scs = AtlasBehaviorSimulation.createForAutomatedTest(robotModel, new FlatGroundEnvironment());
      variables = new AtlasBehaviorTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AtlasTestScripts.standUp(conductor, variables);
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(AtlasPatrolBehaviorTest.class + " after class.");
   }
}
