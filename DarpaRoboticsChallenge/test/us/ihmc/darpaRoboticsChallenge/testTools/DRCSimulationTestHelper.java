package us.ihmc.darpaRoboticsChallenge.testTools;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandstepPacket;
import us.ihmc.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
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
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.BoundingBox3d;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.yoUtilities.time.GlobalTimer;

public class DRCSimulationTestHelper
{
   private final SimulationConstructionSet scs;
   private final SDFRobot sdfRobot;
   private final DRCSimulationFactory drcSimulationFactory;
   private final PacketCommunicator controllerCommunicator;
   private final CommonAvatarEnvironmentInterface testEnvironment;

   private final SimulationTestingParameters simulationTestingParameters;

   private final NothingChangedVerifier nothingChangedVerifier;
   private BlockingSimulationRunner blockingSimulationRunner;
   private final WalkingControllerParameters walkingControlParameters;


   private final FullRobotModel fullRobotModel;
   private final ReferenceFrames referenceFrames;
   private final ScriptedFootstepGenerator scriptedFootstepGenerator;
   private final ScriptedHandstepGenerator scriptedHandstepGenerator;

   private final DRCNetworkModuleParameters networkProcessorParameters;
  
   public DRCSimulationTestHelper(String name, String scriptFileName, DRCObstacleCourseStartingLocation selectedLocation, SimulationTestingParameters simulationconstructionsetparameters, DRCRobotModel robotModel)
   {
      this(new DRCDemo01NavigationEnvironment(), new ScriptedFootstepDataListObjectCommunicator("Team"), name, scriptFileName, selectedLocation, simulationconstructionsetparameters, false, robotModel);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, String scriptFileName, DRCStartingLocation selectedLocation,
         SimulationTestingParameters simulationConstructionSetParameters, DRCRobotModel robotModel)
   {
      this(commonAvatarEnvironmentInterface, new ScriptedFootstepDataListObjectCommunicator("Team"), name, scriptFileName, selectedLocation, simulationConstructionSetParameters, false, robotModel);
   }
   
         
   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, PacketCommunicator packetCommunicator, String name, 
         String scriptFileName, DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters , boolean startNetworkProcessor, DRCRobotModel robotModel)
   {
      this.controllerCommunicator = packetCommunicator;
      this.testEnvironment = commonAvatarEnvironmentInterface;

      this.walkingControlParameters = robotModel.getWalkingControllerParameters();

      this.simulationTestingParameters = simulationTestingParameters;

      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new ReferenceFrames(fullRobotModel);
      scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);
      scriptedHandstepGenerator = new ScriptedHandstepGenerator(fullRobotModel);

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, commonAvatarEnvironmentInterface);
      simulationStarter.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationStarter.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());

      simulationStarter.setScriptFile(scriptFileName);
      simulationStarter.setStartingLocation(selectedLocation);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);
      
      if (startNetworkProcessor)
      {
         networkProcessorParameters = new DRCNetworkModuleParameters();
         networkProcessorParameters.setUseSensorModule(startNetworkProcessor);
         networkProcessorParameters.setUsePerceptionModule(startNetworkProcessor);
         networkProcessorParameters.setUseBehaviorVisualizer(false); // Needs to be false for bamboo.
      }
      else
      {
         networkProcessorParameters = null;
         simulationStarter.setControllerPacketCommunicator(packetCommunicator);
      }
      
      simulationStarter.startSimulation(networkProcessorParameters, false);

      scs = simulationStarter.getSimulationConstructionSet();
      sdfRobot = simulationStarter.getSDFRobot();
      drcSimulationFactory = simulationStarter.getDRCSimulationFactory();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
      simulationStarter.attachControllerFailureListener(blockingSimulationRunner.createControllerFailureListener());

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
   
   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      MomentumBasedControllerFactory controllerFactory = drcSimulationFactory.getControllerFactory();
      controllerFactory.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }
   
   public SDFFullRobotModel getControllerFullRobotModel()
   {
      return drcSimulationFactory.getControllerFullRobotModel();
   }
   
   public SDFFullRobotModel getSDFFullRobotModel()
   {
      return (SDFFullRobotModel) fullRobotModel;
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

         ArrayList<String> stringsToIgnore = createVariableNamesStringsToIgnore();

         boolean writeNewBaseFile = nothingChangedVerifier.getWriteNewBaseFile();

         double maxPercentDifference = 0.001;
         nothingChangedVerifier.verifySameResultsAsPreviously(maxPercentDifference, stringsToIgnore);
         assertFalse("Had to write new base file. On next run nothing should change", writeNewBaseFile);
      }
   }

   public void sendFootstepListToListeners(FootstepDataList footstepDataList)
   {
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendFootstepListToListeners(footstepDataList);
   }
   
   public void sendHandstepPacketToListeners(HandstepPacket handstepPacket)
   {
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendHandstepPacketToListeners(handstepPacket);
   }
   
   public void sendHandPosePacketToListeners(HandPosePacket handPosePacket)
   {
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendHandPosePacketToListeners(handPosePacket);
      if (controllerCommunicator instanceof KryoLocalPacketCommunicator)
         controllerCommunicator.send(handPosePacket);
   }

   public void sendBlindWalkingPacketToListeners(BlindWalkingPacket blindWalkingPacket)
   {
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendBlindWalkingPacketToListeners(blindWalkingPacket);
   }

   public void sendComHeightPacketToListeners(ComHeightPacket comHeightPacket)
   {
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendComHeightPacketToListeners(comHeightPacket);
   }

   public void sendChestOrientationPacketToListeners(ChestOrientationPacket chestOrientationPacket)
   {
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendChestOrientation(chestOrientationPacket);
   }
   
   public void sendWholeBodyTrajectoryPacketToListeners(WholeBodyTrajectoryPacket wholeBodyTrajectoryPacket){
      if (controllerCommunicator instanceof ScriptedFootstepDataListObjectCommunicator)
         ((ScriptedFootstepDataListObjectCommunicator) controllerCommunicator).sendWholeBodyTrajectory(wholeBodyTrajectoryPacket);
   }

   public SDFRobot getRobot()
   {
      return sdfRobot;
   }

   public void simulateAndBlock(double simulationTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
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
      
      
      
      if (networkProcessorParameters != null)
      {
         PacketCommunicator simulatedSensorCommunicator = networkProcessorParameters.getSimulatedSensorCommunicator();
         if (simulatedSensorCommunicator != null)
         {
            simulatedSensorCommunicator.close();
         }
         
         PacketCommunicator controllerCommunicator = networkProcessorParameters.getControllerCommunicator();
         if (controllerCommunicator != null)
         {
            controllerCommunicator.close();
         }
      }
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

   public static ArrayList<String> createVariableNamesStringsToIgnore()
   {
      ArrayList<String> exceptions = new ArrayList<String>();
      exceptions.add("nano");
      exceptions.add("milli");
      exceptions.add("Timer");
      exceptions.add("startTime");
      exceptions.add("actualEstimatorDT");
      exceptions.add("nextExecutionTime");
      exceptions.add("totalDelay");
      exceptions.add("lastEstimatorClockStartTime");
      exceptions.add("lastControllerClockTime");
      exceptions.add("controllerStartTime");
      exceptions.add("actualControlDT");
      exceptions.add("timePassed");
      
//    exceptions.add("gc_");
//    exceptions.add("toolFrame");
//    exceptions.add("ef_");
//    exceptions.add("kp_");

      return exceptions;
   }
}
