package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.WalkOverTerrainStateMachineBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CinderBlockFieldWithFiducialEnvironment;
import us.ihmc.simulationconstructionset.util.environments.CinderBlockFieldWithFiducialEnvironment.FiducialType;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarWalkOverTerrainBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private DRCRobotModel drcRobotModel;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCWalkToLocationBehaviorTest.class + " after class.");
   }

   @Before
   public void setUp()
   {
      drcRobotModel = getRobotModel();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(new CinderBlockFieldWithFiducialEnvironment(FiducialType.FIDUCIAL_50), getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, drcRobotModel);
   }

   public void testWalkOverCinderBlocks() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         simulationTestingParameters.setKeepSCSUp(true);
      }

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      CommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();
      FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(communicationBridge, yoGraphicsListRegistry);
      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      YoVariableRegistry scsRootRegistry = drcBehaviorTestHelper.getSimulationConstructionSet().getRootRegistry();

      AtlasPrimitiveActions primitiveActions = new AtlasPrimitiveActions(communicationBridge, fullRobotModel, referenceFrames, drcRobotModel.getWalkingControllerParameters(), yoTime, drcRobotModel, scsRootRegistry);

      LogModelProvider logModelProvider = drcRobotModel.getLogModelProvider();
      
      WalkOverTerrainStateMachineBehavior walkOverTerrainBehavior = new WalkOverTerrainStateMachineBehavior(communicationBridge, yoTime, primitiveActions, 
            logModelProvider, fullRobotModel, referenceFrames, fiducialDetectorBehaviorService);
      walkOverTerrainBehavior.initialize();
      drcBehaviorTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);

      drcBehaviorTestHelper.addChildRegistry(fiducialDetectorBehaviorService.getYoVariableRegistry());
      drcBehaviorTestHelper.addChildRegistry(walkOverTerrainBehavior.getYoVariableRegistry());

      drcBehaviorTestHelper.getAvatarSimulation().start();
//      drcBehaviorTestHelper.getAvatarSimulation().simulate();

      assertTrue(drcBehaviorTestHelper.executeBehaviorUntilDone(walkOverTerrainBehavior));
      assertTrue(walkOverTerrainBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
}
