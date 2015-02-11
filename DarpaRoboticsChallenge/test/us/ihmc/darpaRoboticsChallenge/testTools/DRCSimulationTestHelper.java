package us.ihmc.darpaRoboticsChallenge.testTools;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandstepPacket;
import us.ihmc.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.BoundingBox3d;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.time.GlobalTimer;

public class DRCSimulationTestHelper
{
   private final SimulationConstructionSet scs;
   private final SDFRobot sdfRobot;
   private final DRCSimulationFactory drcSimulationFactory;
   private final PacketCommunicator networkObjectCommunicator;
   private final CommonAvatarEnvironmentInterface testEnvironment;

   private final SimulationTestingParameters simulationTestingParameters;

   private final NothingChangedVerifier nothingChangedVerifier;
   private BlockingSimulationRunner blockingSimulationRunner;
   private final WalkingControllerParameters walkingControlParameters;


   private final FullRobotModel fullRobotModel;
   private final ReferenceFrames referenceFrames;
   private final ScriptedFootstepGenerator scriptedFootstepGenerator;
   private final ScriptedHandstepGenerator scriptedHandstepGenerator;

  
   public DRCSimulationTestHelper(String name, String scriptFileName, DRCObstacleCourseStartingLocation selectedLocation, SimulationTestingParameters simulationconstructionsetparameters, DRCRobotModel robotModel)
   {
      this(new DRCDemo01NavigationEnvironment(), new ScriptedFootstepDataListObjectCommunicator("Team"), name, scriptFileName, selectedLocation, simulationconstructionsetparameters, false, robotModel);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, String scriptFileName, DRCStartingLocation selectedLocation,
         SimulationTestingParameters simulationConstructionSetParameters, DRCRobotModel robotModel)
   {
      this(commonAvatarEnvironmentInterface, new ScriptedFootstepDataListObjectCommunicator("Team"), name, scriptFileName, selectedLocation, simulationConstructionSetParameters, false, robotModel);
   }
   
         
   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, PacketCommunicator networkObjectCommunicator, String name, 
         String scriptFileName, DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters , boolean startNetworkProcessor, DRCRobotModel robotModel)
   {
      this.networkObjectCommunicator = networkObjectCommunicator;
      this.testEnvironment = commonAvatarEnvironmentInterface;

      this.walkingControlParameters = robotModel.getWalkingControllerParameters();

      this.simulationTestingParameters = simulationTestingParameters;

      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new ReferenceFrames(fullRobotModel);
      scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);
      scriptedHandstepGenerator = new ScriptedHandstepGenerator(fullRobotModel);

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, commonAvatarEnvironmentInterface);
      simulationStarter.setScriptFile(scriptFileName);
      simulationStarter.setStartingLocation(selectedLocation);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);
      
      
      
      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      if (startNetworkProcessor)
      {
         networkProcessorParameters.setUseSensorModule(startNetworkProcessor);
         networkProcessorParameters.setUsePerceptionModule(startNetworkProcessor);
         
         simulationStarter.addPacketCommunicatorToNetworkProcessor(networkObjectCommunicator);
      }
      
      simulationStarter.setControllerPacketCommunicator(networkObjectCommunicator);
      simulationStarter.startSimulation(networkProcessorParameters, false);

      scs = simulationStarter.getSimulationConstructionSet();
      sdfRobot = simulationStarter.getSDFRobot();
      drcSimulationFactory = simulationStarter.getDRCSimulationFactory();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         nothingChangedVerifier = new NothingChangedVerifier(name, scs);
      }
      else
      {
         nothingChangedVerifier = null;
      }
   }


   public BlockingSimulationRunner getBlockingSimulationRunner()
   {
      return blockingSimulationRunner;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }
   
   public DRCSimulationFactory getDRCSimulationFactory()
   {
      return drcSimulationFactory;
   }
   
   public CommonAvatarEnvironmentInterface getTestEnviroment()
   {
      return testEnvironment;
   }

   public ScriptedFootstepGenerator createScriptedFootstepGenerator()
   {
      return scriptedFootstepGenerator;
   }
   
   public ScriptedHandstepGenerator createScriptedHandstepGenerator()
   {
      return scriptedHandstepGenerator;
   }

   public void checkNothingChanged()
   {
      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         ThreadTools.sleep(1000);

         ArrayList<String> stringsToIgnore = new ArrayList<String>();
         stringsToIgnore.add("nano");
         stringsToIgnore.add("milli");
         stringsToIgnore.add("Timer");
         stringsToIgnore.add("startTime");
         stringsToIgnore.add("actualEstimatorDT");
         stringsToIgnore.add("nextExecutionTime");
         stringsToIgnore.add("totalDelay");
         stringsToIgnore.add("lastEstimatorClockStartTime");
         stringsToIgnore.add("lastControllerClockTime");
         stringsToIgnore.add("controllerStartTime");
         stringsToIgnore.add("actualControlDT");

         boolean writeNewBaseFile = nothingChangedVerifier.getWriteNewBaseFile();

         double maxPercentDifference = 0.001;
         nothingChangedVerifier.verifySameResultsAsPreviously(maxPercentDifference, stringsToIgnore);
         assertFalse("Had to write new base file. On next run nothing should change", writeNewBaseFile);
      }
   }

   public void sendFootstepListToListeners(FootstepDataList footstepDataList)
   {
      if (networkObjectCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) networkObjectCommunicator).sendFootstepListToListeners(footstepDataList);
   }
   
   public void sendHandstepPacketToListeners(HandstepPacket handstepPacket)
   {
      if (networkObjectCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) networkObjectCommunicator).sendHandstepPacketToListeners(handstepPacket);
   }
   
   public void sendHandPosePacketToListeners(HandPosePacket handPosePacket)
   {
      if (networkObjectCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) networkObjectCommunicator).sendHandPosePacketToListeners(handPosePacket);

   }

   public void sendBlindWalkingPacketToListeners(BlindWalkingPacket blindWalkingPacket)
   {
      if (networkObjectCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) networkObjectCommunicator).sendBlindWalkingPacketToListeners(blindWalkingPacket);
   }

   public void sendComHeightPacketToListeners(ComHeightPacket comHeightPacket)
   {
      if (networkObjectCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) networkObjectCommunicator).sendComHeightPacketToListeners(comHeightPacket);
   }

   public void sendChestOrientationPacketToListeners(ChestOrientationPacket chestOrientationPacket)
   {
      if (networkObjectCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) networkObjectCommunicator).sendChestOrientation(chestOrientationPacket);
   }

   public SDFRobot getRobot()
   {
      return sdfRobot;
   }

   public void simulateAndBlock(double simulationTime) throws SimulationExceededMaximumTimeException
   {
      blockingSimulationRunner.simulateAndBlock(simulationTime);
   }

   public void destroySimulation()
   {
      blockingSimulationRunner.destroySimulation();
      blockingSimulationRunner = null;
      if (drcSimulationFactory != null)
      {
         drcSimulationFactory.dispose();
      }
      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();
   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime) throws SimulationExceededMaximumTimeException
   {
      try
      {
         simulateAndBlock(simulationTime);
         return true;
      }
      catch (Exception e)
      {
         System.err.println("Caught exception in " + getClass().getSimpleName() + ".simulateAndBlockAndCatchExceptions. Exception = /n" + e);
         return false;
      }
   }

   public void createMovie(String simplifiedRobotModelName, int callStackHeight)
   {
      if (simulationTestingParameters.getCreateSCSMovies())
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName, scs, callStackHeight);
      }
   }

   public RobotSide[] createRobotSidesStartingFrom(RobotSide robotSide, int length)
   {
      RobotSide[] ret = new RobotSide[length];

      for (int i = 0; i < length; i++)
      {
         ret[i] = robotSide;
         robotSide = robotSide.getOppositeSide();
      }

      return ret;
   }

   public void setupCameraForUnitTest(Point3d cameraFix, Point3d cameraPosition)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");

      Random randomForSlightlyMovingCameraSoThatYouTubeVideosAreDifferent = new Random();
      Vector3d randomCameraOffset = RandomTools.generateRandomVector(randomForSlightlyMovingCameraSoThatYouTubeVideosAreDifferent, 0.05);
      cameraFix.add(randomCameraOffset);

      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(false, true, true, false);
      cameraConfiguration.setCameraDolly(false, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   
   public void assertRobotsRootJointIsInBoundingBox(BoundingBox3d boundingBox)
   {
      FloatingJoint rootJoint = getRobot().getRootJoint();
      Point3d position = new Point3d();
      rootJoint.getPosition(position);
      boolean inside = boundingBox.isInside(position);
      if (!inside)
      {
         fail("Joint was at " + position + ". Expecting it to be inside boundingBox " + boundingBox);
      }
   }


}
