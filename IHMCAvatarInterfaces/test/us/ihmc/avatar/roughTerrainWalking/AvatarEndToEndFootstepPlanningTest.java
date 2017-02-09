package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Arrays;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

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
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ConstantGoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.AnytimePlannerStateMachineBehavior;
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
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarEndToEndFootstepPlanningTest implements MultiRobotTestInterface
{
   private static final boolean LOCAL_MODE = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private CommunicationBridge communicationBridge;
   private DoubleYoVariable yoTime;

   private FullHumanoidRobotModel fullRobotModel;

   private HumanoidRobotDataReceiver robotDataReceiver;
   private BehaviorDispatcher<HumanoidBehaviorType> behaviorDispatcher;

   private PacketCommunicator behaviorCommunicatorServer;
   private PacketCommunicator behaviorCommunicatorClient;

   private PacketRouter<PacketDestination> networkProcessor;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private HumanoidReferenceFrames referenceFrames;
   private YoVariableRegistry registry;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      networkProcessor = new PacketRouter<>(PacketDestination.class);
      registry = new YoVariableRegistry(getClass().getSimpleName());
      this.yoTime = new DoubleYoVariable("yoTime", registry);

      behaviorCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      behaviorCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      try
      {
         behaviorCommunicatorClient.connect();
         behaviorCommunicatorServer.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      this.communicationBridge = new CommunicationBridge(behaviorCommunicatorServer);
      networkProcessor.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorCommunicatorClient);

      fullRobotModel = getRobotModel().createFullRobotModel();
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, communicationBridge, yoGraphicsListRegistry, behaviorCommunicatorServer, registry);
      referenceFrames = robotDataReceiver.getReferenceFrames();
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

      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber,
                                                                              desiredBehaviorSubscriber, communicationBridge, yoVariableServer,
                                                                              HumanoidBehaviorType.class, HumanoidBehaviorType.STOP, registry,
                                                                              yoGraphicsListRegistry);

      return ret;
   }

   private void setUpSimulationTestHelper(CommonAvatarEnvironmentInterface environment, DRCObstacleCourseStartingLocation startingLocation)
   {
      if(drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
      }

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getSimpleRobotName(), startingLocation, simulationTestingParameters, getRobotModel());
      networkProcessor.attachPacketCommunicator(PacketDestination.CONTROLLER, drcSimulationTestHelper.getControllerCommunicator());
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

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testAnytimePlannerBehaviorOverRoughTerrain() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PlanarRegionsList cinderBlockFieldPlanarRegions = PlanarRegionsListExamples.generateCinderBlockField(0.0, 0.0, 0.5, 0.05, 5, 4, 0.0);
      PlanarRegionsListDefinedEnvironment cinderBlockFieldEnvironment = new PlanarRegionsListDefinedEnvironment(cinderBlockFieldPlanarRegions, 0.02);
      setUpSimulationTestHelper(cinderBlockFieldEnvironment, DRCObstacleCourseStartingLocation.DEFAULT);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d goalPoint = new Point3d(3.0, 0.0, 0.0);
      GoalDetectorBehaviorService goalDetectorBehaviorService = new ConstantGoalDetectorBehaviorService(referenceFrames, goalPoint,
                                                                                                        communicationBridge);
      boolean createYoVariableServerForPlannerVisualizer = LOCAL_MODE;
      AnytimePlannerStateMachineBehavior walkOverTerrainStateMachineBehavior = new AnytimePlannerStateMachineBehavior(communicationBridge, yoTime,
                                                                                                                      referenceFrames,
                                                                                                                      getRobotModel().getLogModelProvider(),
                                                                                                                      fullRobotModel, getRobotModel(),
                                                                                                                      yoGraphicsListRegistry, goalDetectorBehaviorService, createYoVariableServerForPlannerVisualizer);

      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_GOAL_ANYTIME_PLANNER, walkOverTerrainStateMachineBehavior);
      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_GOAL_ANYTIME_PLANNER);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockFieldPlanarRegions);
      planarRegionsListMessage.setDestination(PacketDestination.BROADCAST);

      for (int i = 0; i < 4; i++)
      {
         // allow 20sec for each set of 5 steps
         behaviorCommunicatorClient.send(planarRegionsListMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);
         assertTrue(success);
      }

      Point2d planarGoalPoint = new Point2d(goalPoint.getX(), goalPoint.getY());
      // TODO figure out a better assertion method, the anytime planner uses a threshold of 0.5
      assertBodyIsCloseToXYLocation(planarGoalPoint, 0.6);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testAnytimeBehaviorOverIncrementalTerrain() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      PlanarRegionsList stairsUp = PlanarRegionsListExamples.generateStairCase();
      PlanarRegionsList stairsDown = PlanarRegionsListExamples.generateStairCase(new Vector3d(3.8, 0.0, 0.0), new Vector3d(Math.PI, Math.PI, 0.0));
      PlanarRegionsList completeStairCase = new PlanarRegionsList();

      for(int i = 0; i < stairsUp.getNumberOfPlanarRegions(); i++)
      {
         completeStairCase.addPlanarRegion(stairsUp.getPlanarRegion(i));
         completeStairCase.addPlanarRegion(stairsDown.getPlanarRegion(i));
      }

      PlanarRegionsListDefinedEnvironment stairCaseEnvironment = new PlanarRegionsListDefinedEnvironment(completeStairCase, 0.02);
      setUpSimulationTestHelper(stairCaseEnvironment, DRCObstacleCourseStartingLocation.DEFAULT);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d goalPoint = new Point3d(4.0, 0.0, 0.0);
      GoalDetectorBehaviorService goalDetectorBehaviorService = new ConstantGoalDetectorBehaviorService(referenceFrames, goalPoint,
                                                                                                        communicationBridge);
      boolean createYoVariableServerForPlannerVisualizer = LOCAL_MODE;
      AnytimePlannerStateMachineBehavior walkOverTerrainStateMachineBehavior = new AnytimePlannerStateMachineBehavior(communicationBridge, yoTime,
                                                                                                                      referenceFrames,
                                                                                                                      getRobotModel().getLogModelProvider(),
                                                                                                                      fullRobotModel, getRobotModel(),
                                                                                                                      yoGraphicsListRegistry, goalDetectorBehaviorService,
                                                                                                                      createYoVariableServerForPlannerVisualizer);

      behaviorDispatcher.addBehavior(HumanoidBehaviorType.WALK_TO_GOAL_ANYTIME_PLANNER, walkOverTerrainStateMachineBehavior);
      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestWalkToObjectBehaviorPacket = new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_TO_GOAL_ANYTIME_PLANNER);
      behaviorCommunicatorClient.send(requestWalkToObjectBehaviorPacket);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PlanarRegionsListMessage stairsUpPlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(stairsUp);
      stairsUpPlanarRegionsListMessage.setDestination(PacketDestination.BROADCAST);
      PlanarRegionsListMessage completeStairCasePlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(completeStairCase);
      completeStairCasePlanarRegionsListMessage.setDestination(PacketDestination.BROADCAST);

      for (int i = 0; i < 6; i++)
      {
         // allow 20sec for each set of 5 steps
         if(i < 2)
         {
            behaviorCommunicatorClient.send(stairsUpPlanarRegionsListMessage);
         }
         else
         {
            behaviorCommunicatorClient.send(completeStairCasePlanarRegionsListMessage);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);
         assertTrue(success);
      }

      Point2d planarGoalPoint = new Point2d(goalPoint.getX(), goalPoint.getY());
      // TODO figure out a better assertion method, the anytime planner uses a threshold of 0.5
      assertBodyIsCloseToXYLocation(planarGoalPoint, 0.6);
   }

   private void assertBodyIsCloseToXYLocation(Point2d point, double threshold)
   {
      referenceFrames.updateFrames();
      ReferenceFrame bodyFrame = referenceFrames.getABodyAttachedZUpFrame();
      FramePoint goalPoint = new FramePoint(ReferenceFrame.getWorldFrame(), point.getX(), point.getY(), 0.0);
      goalPoint.changeFrame(bodyFrame);

      double distanceFrameBody = Math.sqrt(goalPoint.getX() * goalPoint.getX() + goalPoint.getY() * goalPoint.getY());
      assertTrue("Robot is not within " + threshold + "m of goal", distanceFrameBody < threshold);
   }
}
