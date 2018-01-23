package us.ihmc.avatar.testTools;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.obstacleCourseTests.ForceSensorHysteresisCreator;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.simulationTesting.NothingChangedVerifier;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class DRCSimulationTestHelper
{
   private SimulationConstructionSet scs;
   private HumanoidFloatingRootJointRobot sdfRobot;
   private AvatarSimulation avatarSimulation;

   protected final PacketCommunicator controllerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
         new IHMCCommunicationKryoNetClassList());
   private CommonAvatarEnvironmentInterface testEnvironment = new DefaultCommonAvatarEnvironment();

   private final SimulationTestingParameters simulationTestingParameters;

   private NothingChangedVerifier nothingChangedVerifier;
   private BlockingSimulationRunner blockingSimulationRunner;
   private final WalkingControllerParameters walkingControlParameters;

   private final DRCRobotModel robotModel;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ScriptedFootstepGenerator scriptedFootstepGenerator;
   private final ScriptedHandstepGenerator scriptedHandstepGenerator;

   private DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
   private DRCSimulationStarter simulationStarter;
   private Exception caughtException;

   private OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
   private boolean addFootstepMessageGenerator = false;
   private boolean useHeadingAndVelocityScript = false;
   private boolean cheatWithGroundHeightAtFootstep = false;
   private HighLevelControllerStateFactory controllerStateFactory = null;
   private ControllerStateTransitionFactory<HighLevelControllerName> controllerStateTransitionFactory = null;
   private DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup = null;
   private HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = null;
   private final DRCGuiInitialSetup guiInitialSetup;

   private final boolean checkIfDesiredICPHasBeenInvalid = true;

   public DRCSimulationTestHelper(SimulationTestingParameters simulationTestParameters, DRCRobotModel robotModel)
   {
      this(simulationTestParameters, robotModel, null);
   }

   public DRCSimulationTestHelper(SimulationTestingParameters simulationTestParameters, DRCRobotModel robotModel, CommonAvatarEnvironmentInterface testEnvironment)
   {
      this.robotModel = robotModel;
      this.walkingControlParameters = robotModel.getWalkingControllerParameters();
      this.simulationTestingParameters = simulationTestParameters;

      if (testEnvironment != null)
         this.testEnvironment = testEnvironment;
      simulationStarter = new DRCSimulationStarter(robotModel, this.testEnvironment);

      fullRobotModel = robotModel.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);
      scriptedHandstepGenerator = new ScriptedHandstepGenerator(fullRobotModel);

      guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      networkProcessorParameters.enableNetworkProcessor(false);
   }

   /**
    * Use {@link #DRCSimulationTestHelper(SimulationTestingParameters, DRCRobotModel, CommonAvatarEnvironmentInterface)} instead.
    */
   @Deprecated
   public void setTestEnvironment(CommonAvatarEnvironmentInterface testEnvironment)
   {
      this.testEnvironment = testEnvironment;
      simulationStarter = new DRCSimulationStarter(robotModel, testEnvironment);
   }

   public void createSimulation(String name)
   {
      createSimulation(name, true, true);
   }

   public void createSimulation(String name, boolean automaticallySpawnSimulation, boolean useBlockingSimulationRunner)
   {
      simulationStarter.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationStarter.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());
      if (controllerStateFactory != null)
         simulationStarter.registerHighLevelControllerState(controllerStateFactory);
      if (controllerStateTransitionFactory != null)
         simulationStarter.registerControllerStateTransition(controllerStateTransitionFactory);
      if (initialSetup != null)
         simulationStarter.setRobotInitialSetup(initialSetup);
      simulationStarter.setStartingLocationOffset(startingLocation);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setFlatGroundWalkingScriptParameters(walkingScriptParameters);

      if (addFootstepMessageGenerator)
         simulationStarter.addFootstepMessageGenerator(useHeadingAndVelocityScript, cheatWithGroundHeightAtFootstep);

      try
      {
         controllerCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      simulationStarter.createSimulation(networkProcessorParameters, automaticallySpawnSimulation, false);

      scs = simulationStarter.getSimulationConstructionSet();
      sdfRobot = simulationStarter.getSDFRobot();
      avatarSimulation = simulationStarter.getAvatarSimulation();
      if (useBlockingSimulationRunner)
      {
         blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
         simulationStarter.attachControllerFailureListener(blockingSimulationRunner.createControllerFailureListener());
         blockingSimulationRunner.createValidDesiredICPListener();
         blockingSimulationRunner.setCheckDesiredICPPosition(checkIfDesiredICPHasBeenInvalid);
      }

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         nothingChangedVerifier = new NothingChangedVerifier(name, scs);
      }
      else
      {
         nothingChangedVerifier = null;
      }
   }

   public YoVariable<?> getYoVariable(String name)
   {
      return scs.getVariable(name);
   }

   public YoVariable<?> getYoVariable(String nameSpace, String name)
   {
      return scs.getVariable(nameSpace, name);
   }

   public void loadScriptFile(InputStream scriptInputStream, ReferenceFrame referenceFrame)
   {
      ScriptBasedControllerCommandGenerator scriptBasedControllerCommandGenerator = simulationStarter.getScriptBasedControllerCommandGenerator();
      scriptBasedControllerCommandGenerator.loadScriptFile(scriptInputStream, referenceFrame);
   }

   public DRCSCSInitialSetup getSCSInitialSetup()
   {
      return simulationStarter.getSCSInitialSetup();
   }

   public ConcurrentLinkedQueue<Command<?, ?>> getQueuedControllerCommands()
   {
      return simulationStarter.getQueuedControllerCommands();
   }

   public BlockingSimulationRunner getBlockingSimulationRunner()
   {
      return blockingSimulationRunner;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      HighLevelHumanoidControllerFactory controllerFactory = avatarSimulation.getHighLevelHumanoidControllerFactory();
      controllerFactory.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarSimulation.getControllerFullRobotModel();
   }

   public FullHumanoidRobotModel getSDFFullRobotModel()
   {
      return fullRobotModel;
   }

   public CommonHumanoidReferenceFrames getReferenceFrames()
   {
      HighLevelHumanoidControllerFactory momentumBasedControllerFactory = avatarSimulation.getHighLevelHumanoidControllerFactory();
      HighLevelHumanoidControllerToolbox highLevelHumanoidControllerToolbox = momentumBasedControllerFactory.getHighLevelHumanoidControllerToolbox();
      return highLevelHumanoidControllerToolbox.getReferenceFrames();
   }

   /**
    * For unit testing only
    *
    * @param controller
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnControllerThread(controller);
   }

   /**
    * For unit testing only
    *
    * @param controller
    */
   public void addRobotControllerOnEstimatorThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnEstimatorThread(controller);
   }

   public CommonAvatarEnvironmentInterface getTestEnvironment()
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

   public HumanoidFloatingRootJointRobot getRobot()
   {
      return sdfRobot;
   }

   public void simulateAndBlock(double simulationTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.simulateAndBlock(simulationTime);
      }
   }

   public void destroySimulation()
   {
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
      }
      blockingSimulationRunner = null;
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
      }

      if (networkProcessorParameters != null)
      {
         LocalObjectCommunicator simulatedSensorCommunicator = networkProcessorParameters.getSimulatedSensorCommunicator();
         if (simulatedSensorCommunicator != null)
         {
            simulatedSensorCommunicator.disconnect();
         }

      }
      if (controllerCommunicator != null)
      {
         controllerCommunicator.disconnect();
      }

      simulationStarter.close();
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
         this.caughtException = e;
         PrintTools.error(this, e.getMessage());
         return false;
      }
   }

   public void addChildRegistry(YoVariableRegistry childRegistry)
   {
      scs.getRootRegistry().addChild(childRegistry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return scs.getRootRegistry();
   }

   public void createVideo(String simplifiedRobotModelName, int callStackHeight)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName, scs, callStackHeight);
      }
   }
   
   public void createVideo(String videoName)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeAndStoreInDefaultDirectory(scs, videoName);
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

   public void setupCameraForUnitTest(Point3D cameraFix, Point3D cameraPosition)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");

      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(false, true, true, false);
      cameraConfiguration.setCameraDolly(false, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   public void setupCameraForUnitTest(boolean enableTracking, Point3D cameraFix, Point3D cameraPosition)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      
      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(enableTracking, true, true, false);
      cameraConfiguration.setCameraDolly(false, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   public void assertRobotsRootJointIsInBoundingBox(BoundingBox3D boundingBox)
   {
      assertRobotsRootJointIsInBoundingBox(boundingBox, getRobot());
   }

   public static void assertRobotsRootJointIsInBoundingBox(BoundingBox3D boundingBox, HumanoidFloatingRootJointRobot robot)
   {
      Point3D position = new Point3D();
      robot.getRootJoint().getPosition(position);
      boolean inside = boundingBox.isInsideInclusive(position);
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

   public Exception getCaughtException()
   {
      return caughtException;
   }

   public void send(Packet<?> packet)
   {
      controllerCommunicator.send(packet);
   }

   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      controllerCommunicator.attachListener(clazz, listener);
   }

   public PacketCommunicator getControllerCommunicator()
   {
      return controllerCommunicator;
   }

   public ArrayList<RobotController> getFootForceSensorHysteresisCreators()
   {
      SideDependentList<ArrayList<WrenchCalculatorInterface>> footForceSensors = new SideDependentList<ArrayList<WrenchCalculatorInterface>>();
      packFootForceSensors(footForceSensors);

      ArrayList<RobotController> footForceSensorSignalCorruptors = new ArrayList<RobotController>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int i = 0; i < footForceSensors.get(robotSide).size(); i++)
         {
            ForceSensorHysteresisCreator forceSensorSignalCorruptor = new ForceSensorHysteresisCreator(sdfRobot.computeCenterOfMass(new Point3D()),
                                                                                                       footForceSensors.get(robotSide).get(i).getName(),
                                                                                                       footForceSensors.get(robotSide).get(i));

            footForceSensorSignalCorruptors.add(forceSensorSignalCorruptor);
         }
      }

      return footForceSensorSignalCorruptors;
   }

   public void packFootForceSensors(SideDependentList<ArrayList<WrenchCalculatorInterface>> footForceSensors)
   {
      ArrayList<WrenchCalculatorInterface> forceSensors = new ArrayList<WrenchCalculatorInterface>();
      sdfRobot.getForceSensors(forceSensors);

      SideDependentList<String> jointNamesBeforeFeet = sdfRobot.getJointNamesBeforeFeet();

      for (RobotSide robotSide : RobotSide.values)
      {
         footForceSensors.put(robotSide, new ArrayList<WrenchCalculatorInterface>());
         for (int i = 0; i < forceSensors.size(); i++)
         {
            if (forceSensors.get(i).getJoint().getName().equals(jointNamesBeforeFeet.get(robotSide)))
            {
               footForceSensors.get(robotSide).add(forceSensors.get(i));
            }
         }
      }
   }

   public void setStartingLocation(DRCStartingLocation startingLocation)
   {
      if (startingLocation != null)
      {
         this.startingLocation = startingLocation.getStartingLocationOffset();
      }
   }

   public void setStartingLocation(OffsetAndYawRobotInitialSetup startingLocation)
   {
      this.startingLocation = startingLocation;
   }

   public void setAddFootstepMessageGenerator(boolean addFootstepMessageGenerator)
   {
      this.addFootstepMessageGenerator = addFootstepMessageGenerator;
   }

   public void setUseHeadingAndVelocityScript(boolean useHeadingAndVelocityScript)
   {
      this.useHeadingAndVelocityScript = useHeadingAndVelocityScript;
   }

   public void setCheatWithGroundHeightAtFootstep(boolean cheatWithGroundHeightAtFootstep)
   {
      this.cheatWithGroundHeightAtFootstep = cheatWithGroundHeightAtFootstep;
   }

   public void setHighLevelControllerStateFactory(HighLevelControllerStateFactory controllerStateFactory)
   {
      this.controllerStateFactory = controllerStateFactory;
   }

   public void setInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup)
   {
      this.initialSetup = initialSetup;
   }

   public void setWalkingScriptParameters(HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      this.walkingScriptParameters = walkingScriptParameters;
   }

   public void setNetworkProcessorParameters(DRCNetworkModuleParameters networkProcessorParameters)
   {
      this.networkProcessorParameters = networkProcessorParameters;
   }
}
