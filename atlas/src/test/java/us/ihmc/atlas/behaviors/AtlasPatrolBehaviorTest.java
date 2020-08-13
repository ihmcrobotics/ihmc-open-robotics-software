package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

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

   @Disabled
   @Test
   public void testPatrolBehavior() throws IOException
   {
      FootstepPlanningModuleLauncher.createModule(robotModel, PubSubImplementation.INTRAPROCESS);

      LogTools.info("Creating behavior module");
      BehaviorModule behaviorModule = BehaviorModule.createIntraprocess(BehaviorRegistry.of(PatrolBehavior.DEFINITION), robotModel);
      Messager messager = behaviorModule.getMessager();

      LogTools.info("Creating behavior messager");
      messager.registerTopicListener(PatrolBehaviorAPI.CurrentState, state -> LogTools.info("Patrol state: {}", state));

      AtlasTestScripts.awaitDuration(conductor, variables, 0.25);  // allows to update frames

      WaypointManager waypointManager = WaypointManager.createForUI(messager,
                                                                    PatrolBehaviorAPI.WaypointsToUI,
                                                                    PatrolBehaviorAPI.WaypointsToModule,
                                                                    () -> {});

      waypointManager.appendNewWaypoint().getPose().set(1.0, -0.2, 0.0, 20.0, 0.0, 0.0);
      waypointManager.appendNewWaypoint().getPose().set(1.0, -1.0, 0.0, 180.0, 0.0, 0.0);
      waypointManager.publish();

      messager.submitMessage(PatrolBehaviorAPI.GoToWaypoint, 0);

      AtlasTestScripts.awaitSteps(conductor, variables, 5, 5.0);
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
      SimulationConstructionSet scs = AtlasDynamicsSimulation.createForAutomatedTest(robotModel, new FlatGroundEnvironment()).getSimulationConstructionSet();
      variables = new AtlasBehaviorTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AtlasTestScripts.awaitStandUp(conductor, variables);
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(AtlasPatrolBehaviorTest.class + " after class.");
   }
}
