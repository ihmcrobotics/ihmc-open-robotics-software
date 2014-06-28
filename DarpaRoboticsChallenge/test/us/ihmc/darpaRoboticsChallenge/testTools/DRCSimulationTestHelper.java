package us.ihmc.darpaRoboticsChallenge.testTools;

import static org.junit.Assert.assertFalse;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.CarIngressEgressControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.packets.ComHeightPacket;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseDemo;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseSimulation;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.scriptEngine.VariousWalkingProviderFromScriptFactory;
import us.ihmc.darpaRoboticsChallenge.visualization.SliderBoardFactory;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import com.yobotics.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;

public class DRCSimulationTestHelper
{
   private final SimulationConstructionSet scs;
   private final SDFRobot sdfRobot;
   private final DRCSimulationFactory drcSimulationFactory;
   private final ScriptedFootstepDataListObjectCommunicator networkObjectCommunicator;

   private final boolean checkNothingChanged;
   private final NothingChangedVerifier nothingChangedVerifier;
   private BlockingSimulationRunner blockingSimulationRunner;
   private final WalkingControllerParameters walkingControlParameters;

   private final boolean createMovie;

   private final DRCRobotModel robotModel;

   public DRCSimulationTestHelper(String name, String scriptFileName, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, boolean checkNothingChanged,
         boolean showGUI, boolean createMovie, DRCRobotModel robotModel, GroundProfile3D groundProfile)
   {
      networkObjectCommunicator = new ScriptedFootstepDataListObjectCommunicator("Team");
      this.walkingControlParameters = robotModel.getWalkingControllerParameters();
      this.robotModel = robotModel;
      this.checkNothingChanged = checkNothingChanged;
      this.createMovie = createMovie;
      if (createMovie)
         showGUI = true;

      boolean groundProfileVisible = !(groundProfile instanceof TerrainObject3D);

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(groundProfileVisible, false, WalkControllerSliderBoard.getFactory(), showGUI);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(false);
//      if (groundProfile != null)
//         robotInitialSetup.setInitialGroundHeight(groundProfile.heightAt(0, 0, 0));

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(walkingControlParameters);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      WalkingControllerParameters multiContactControllerParameters = robotModel.getMultiContactControllerParameters();
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      SideDependentList<String> footForceSensorNames = robotModel.getSensorInformation().getFeetForceSensorNames();
      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
            DRCConfigParameters.contactTresholdForceForSCS, footForceSensorNames, footstepTimingParameters, walkingControllerParameters,
            armControllerParameters, false, false, HighLevelState.WALKING);
      controllerFactory.addHighLevelBehaviorFactory(new CarIngressEgressControllerFactory(multiContactControllerParameters, false));
      controllerFactory.setupForCheatingUsingGroundHeightAtForFootstepProvider(scsInitialSetup.getGroundProfile());
      GlobalDataProducer globalDataProducer = new GlobalDataProducer(networkObjectCommunicator);
      controllerFactory.setupForNetworkedFootstepProvider(globalDataProducer);

      if ((scriptFileName != null) && (!scriptFileName.equals("")))
      {
         VariousWalkingProviderFromScriptFactory variousWalkingProviderFactory = new VariousWalkingProviderFromScriptFactory(scriptFileName);
         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      }
      
      TerrainObject3D environmentTerrain = null;
      if (groundProfile instanceof TerrainObject3D)
         environmentTerrain = (TerrainObject3D) groundProfile;
      
      Graphics3DObject environmentGraphics = environmentTerrain == null ? null : environmentTerrain.getLinkGraphics();
      drcSimulationFactory = new DRCSimulationFactory(robotModel, controllerFactory, environmentGraphics, robotInitialSetup, scsInitialSetup,
            guiInitialSetup, globalDataProducer);
      scs = drcSimulationFactory.getSimulationConstructionSet();
      sdfRobot = drcSimulationFactory.getRobot();
      drcSimulationFactory.start();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);

      if (checkNothingChanged)
      {
         nothingChangedVerifier = new NothingChangedVerifier(name, scs);
      }
      else
      {
         nothingChangedVerifier = null;
      }
   }

   public DRCSimulationTestHelper(String name, String scriptFileName, DRCDemo01StartingLocation selectedLocation, boolean checkNothingChanged, boolean showGUI,
         boolean createMovie, DRCRobotModel robotModel)
   {
      networkObjectCommunicator = new ScriptedFootstepDataListObjectCommunicator("Team");
      this.walkingControlParameters = robotModel.getWalkingControllerParameters();
      this.checkNothingChanged = checkNothingChanged;
      this.createMovie = createMovie;
      this.robotModel = robotModel;
      if (createMovie)
         showGUI = true;

      boolean automaticallyStartSimulation = false;
      boolean startDRCNetworkProcessor = false;

      boolean initializeEstimatorToActual = true;

      SliderBoardFactory sliderBoardFactory = WalkControllerSliderBoard.getFactory();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, sliderBoardFactory, showGUI);

      DRCObstacleCourseSimulation drcSimulation = DRCObstacleCourseDemo.startDRCSim(scriptFileName, networkObjectCommunicator, selectedLocation,
            guiInitialSetup, initializeEstimatorToActual, automaticallyStartSimulation, startDRCNetworkProcessor, false, robotModel);

      scs = drcSimulation.getSimulationConstructionSet();
      sdfRobot = drcSimulation.getRobot();
      drcSimulationFactory = drcSimulation.getSimulation();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);

      if (checkNothingChanged)
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

   public ScriptedFootstepGenerator createScriptedFootstepGenerator()
   {
      FullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel, robotModel.getJointMap(), robotModel.getPhysicalProperties().getAnkleHeight());

      ScriptedFootstepGenerator scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);

      return scriptedFootstepGenerator;
   }

   public void checkNothingChanged()
   {
      if (checkNothingChanged)
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
      networkObjectCommunicator.sendFootstepListToListeners(footstepDataList);
   }

   public void sendFootstepListToListeners(BlindWalkingPacket blindWalkingPacket)
   {
      networkObjectCommunicator.sendBlindWalkingPacketToListeners(blindWalkingPacket);
   }

   public void sendComHeightPacketToListeners(ComHeightPacket comHeightPacket)
   {
      networkObjectCommunicator.sendComHeightPacketToListeners(comHeightPacket);
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
         System.err.println("Caught exception in SimulationTestHelper.simulateAndBlockAndCatchExceptions. Exception = /n" + e);
         throw e;
      }
   }

   public void createMovie(String simplifiedRobotModelName, int callStackHeight)
   {
      if (createMovie)
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

}
