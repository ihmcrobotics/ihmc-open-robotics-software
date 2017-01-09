package us.ihmc.avatar.roughTerrainWalking;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ConstantGoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.AnytimePlannerStateMachineBehavior;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.WalkOverTerrainStateMachineBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import java.io.IOException;
import java.util.Arrays;

import static org.junit.Assert.assertTrue;

public abstract class AvatarEndToEndFootstepPlanningTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private CommunicationBridge communicationBridge;
   private DoubleYoVariable yoTime;

   private FullHumanoidRobotModel fullRobotModel;

   private HumanoidRobotDataReceiver robotDataReceiver;
   private BehaviorDispatcher<HumanoidBehaviorType> behaviorDispatcher;

   private PacketCommunicator behaviorCommunicatorServer;
   private PacketCommunicator behaviorCommunicatorClient;

   private PlanarRegionsList cinderBlockFieldPlanarRegions;

   private AtlasPrimitiveActions atlasPrimitiveActions;
   private HumanoidFloatingRootJointRobot robot;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private HumanoidReferenceFrames referenceFrames;
   private YoVariableRegistry registry;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      PacketRouter<PacketDestination> networkProcessor = new PacketRouter<>(PacketDestination.class);
      registry = new YoVariableRegistry(getClass().getSimpleName());
      this.yoTime = new DoubleYoVariable("yoTime", registry);

      behaviorCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(
            NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      behaviorCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(
            NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      try
      {
         behaviorCommunicatorClient.connect();
         behaviorCommunicatorServer.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      cinderBlockFieldPlanarRegions = PlanarRegionsListExamples.generateCinderBlockField(0.0, 0.0, 0.4, 21, 6);
      PlanarRegionsListDefinedEnvironment cinderBlockFieldEnvironment = new PlanarRegionsListDefinedEnvironment(cinderBlockFieldPlanarRegions, 0.02);

      this.communicationBridge = new CommunicationBridge(behaviorCommunicatorServer);
      drcSimulationTestHelper = new DRCSimulationTestHelper(cinderBlockFieldEnvironment, getSimpleRobotName(),
                                                            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      networkProcessor.attachPacketCommunicator(PacketDestination.CONTROLLER, drcSimulationTestHelper.getControllerCommunicator());
      networkProcessor.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorCommunicatorClient);

      fullRobotModel = getRobotModel().createFullRobotModel();
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, communicationBridge, yoGraphicsListRegistry, behaviorCommunicatorServer, registry);
      referenceFrames = robotDataReceiver.getReferenceFrames();
      robot = drcSimulationTestHelper.getRobot();
      atlasPrimitiveActions = new AtlasPrimitiveActions(communicationBridge, fullRobotModel, referenceFrames, getRobotModel().getWalkingControllerParameters(), yoTime, getRobotModel(), registry);
   }

   private BehaviorDispatcher<HumanoidBehaviorType> setupBehaviorDispatcher(FullHumanoidRobotModel fullRobotModel, CommunicationBridge communicationBridge,
                                                                            YoGraphicsListRegistry yoGraphicsListRegistry, PacketCommunicator behaviorCommunicatorServer, YoVariableRegistry registry)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      behaviorCommunicatorServer.attachListener(RobotConfigurationData.class, robotDataReceiver);

      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      behaviorCommunicatorServer.attachListener(BehaviorControlModePacket.class, desiredBehaviorControlSubscriber);

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      behaviorCommunicatorServer.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
                                                                              communicationBridge, yoVariableServer, HumanoidBehaviorType.class, HumanoidBehaviorType.STOP, registry, yoGraphicsListRegistry);

      return ret;
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      behaviorCommunicatorClient.close();
      behaviorCommunicatorServer.close();

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         behaviorDispatcher.closeAndDispose();
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
         communicationBridge = null;
         yoTime = null;
         fullRobotModel = null;
         robotDataReceiver = null;
         behaviorDispatcher = null;
         referenceFrames = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testAnytimePlannerOverRoughTerrainBehavior() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      GoalDetectorBehaviorService goalDetectorBehaviorService = new ConstantGoalDetectorBehaviorService(referenceFrames, new Point3d(4.0, 0.0, 0.0),
                                                                                                        communicationBridge);
      AnytimePlannerStateMachineBehavior walkOverTerrainStateMachineBehavior = new AnytimePlannerStateMachineBehavior(communicationBridge, yoTime,
                                                                                                                      referenceFrames,
                                                                                                                      getRobotModel().getLogModelProvider(),
                                                                                                                      fullRobotModel, getRobotModel(),
                                                                                                                      goalDetectorBehaviorService);

      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_GOAL_ANYTIME_PLANNER, walkOverTerrainStateMachineBehavior);
      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_GOAL_ANYTIME_PLANNER);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);
      PrintTools.debug(this, "Requesting WalkToGoal");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockFieldPlanarRegions);
      planarRegionsListMessage.setDestination(PacketDestination.BROADCAST);
      behaviorCommunicatorClient.send(planarRegionsListMessage);

      PrintTools.debug(this, "Setting WalkToLocationBehavior Target");

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0);
      assertTrue(success);
   }
}
