package us.ihmc.avatar.testTools;

import static org.junit.Assert.*;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.obstacleCourseTests.ForceSensorHysteresisCreator;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;
import us.ihmc.tools.thread.ThreadTools;

public class DRCSimulationTestHelper
{
   private final SimulationConstructionSet scs;
   private final HumanoidFloatingRootJointRobot sdfRobot;
   private final AvatarSimulation avatarSimulation;
   protected final PacketCommunicator controllerCommunicator;
   private final CommonAvatarEnvironmentInterface testEnvironment;

   private final SimulationTestingParameters simulationTestingParameters;

   private final NothingChangedVerifier nothingChangedVerifier;
   private BlockingSimulationRunner blockingSimulationRunner;
   private final WalkingControllerParameters walkingControlParameters;

   private final FullHumanoidRobotModel fullRobotModel;
   private final ScriptedFootstepGenerator scriptedFootstepGenerator;
   private final ScriptedHandstepGenerator scriptedHandstepGenerator;

   private final DRCNetworkModuleParameters networkProcessorParameters;
   private DRCSimulationStarter simulationStarter;
   private Exception caughtException;

   public DRCSimulationTestHelper(String name, DRCObstacleCourseStartingLocation selectedLocation,
                                  SimulationTestingParameters simulationconstructionsetparameters, DRCRobotModel robotModel)
   {
      this(new DefaultCommonAvatarEnvironment(), name, selectedLocation, simulationconstructionsetparameters, robotModel, true);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, DRCStartingLocation selectedLocation,
                                  SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel)
   {
      this(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, true);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, DRCStartingLocation selectedLocation,
                                  SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel, boolean automaticallySpawnSimulation)
   {
      this(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, null, null, null, false, false, false,
           automaticallySpawnSimulation, null);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, DRCStartingLocation selectedLocation,
                                  SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel, boolean addFootstepMessageGenerator,
                                  boolean useHeadingAndVelocityScript, boolean cheatWithGroundHeightAtForFootstep,
                                  HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      this(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, null, null, null, addFootstepMessageGenerator,
           useHeadingAndVelocityScript, cheatWithGroundHeightAtForFootstep, true, walkingScriptParameters);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, DRCStartingLocation selectedLocation,
                                  SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel,
                                  DRCNetworkModuleParameters drcNetworkModuleParameters)
   {
      this(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, drcNetworkModuleParameters, null, null, false,
           false, false, true, null);
   }

   public DRCSimulationTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, DRCStartingLocation selectedLocation,
                                  SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel,
                                  DRCNetworkModuleParameters drcNetworkModuleParameters, HighLevelBehaviorFactory highLevelBehaviorFactoryToAdd,
                                  DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, boolean addFootstepMessageGenerator,
                                  boolean useHeadingAndVelocityScript, boolean cheatWithGroundHeightAtForFootstep, boolean automaticallySpawnSimulation,
                                  HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      this.controllerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                            new IHMCCommunicationKryoNetClassList());
      this.testEnvironment = commonAvatarEnvironmentInterface;

      this.walkingControlParameters = robotModel.getWalkingControllerParameters();

      this.simulationTestingParameters = simulationTestingParameters;

      try
      {
         controllerCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      fullRobotModel = robotModel.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);
      scriptedHandstepGenerator = new ScriptedHandstepGenerator(fullRobotModel);

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      simulationStarter = new DRCSimulationStarter(robotModel, commonAvatarEnvironmentInterface);
      simulationStarter.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationStarter.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());
      if (highLevelBehaviorFactoryToAdd != null)
         simulationStarter.registerHighLevelController(highLevelBehaviorFactoryToAdd);
      if (initialSetup != null)
         simulationStarter.setRobotInitialSetup(initialSetup);
      if (selectedLocation != null)
         simulationStarter.setStartingLocation(selectedLocation);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setFlatGroundWalkingScriptParameters(walkingScriptParameters);

      if (addFootstepMessageGenerator)
         simulationStarter.addFootstepMessageGenerator(useHeadingAndVelocityScript, cheatWithGroundHeightAtForFootstep);

      if (drcNetworkModuleParameters == null)
      {
         networkProcessorParameters = new DRCNetworkModuleParameters();
         networkProcessorParameters.enableNetworkProcessor(false);
      }
      else
      {
         networkProcessorParameters = drcNetworkModuleParameters;
      }

      simulationStarter.createSimulation(networkProcessorParameters, automaticallySpawnSimulation, false);

      scs = simulationStarter.getSimulationConstructionSet();
      sdfRobot = simulationStarter.getSDFRobot();
      avatarSimulation = simulationStarter.getAvatarSimulation();
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
      MomentumBasedControllerFactory controllerFactory = avatarSimulation.getMomentumBasedControllerFactory();
      controllerFactory.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarSimulation.getControllerFullRobotModel();
   }

   public FullHumanoidRobotModel getSDFFullRobotModel()
   {
      return (FullHumanoidRobotModel) fullRobotModel;
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

   public HumanoidFloatingRootJointRobot getRobot()
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
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
      }

      if (networkProcessorParameters != null)
      {
         LocalObjectCommunicator simulatedSensorCommunicator = networkProcessorParameters.getSimulatedSensorCommunicator();
         if (simulatedSensorCommunicator != null)
         {
            simulatedSensorCommunicator.close();
         }

      }
      if (controllerCommunicator != null)
      {
         controllerCommunicator.close();
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

   public YoVariableRegistry getYovariableRegistry()
   {
      return scs.getRootRegistry();
   }

   public void createVideo(String simplifiedRobotModelName, int callStackHeight)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName, scs, callStackHeight);
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

      Random randomForSlightlyMovingCameraSoThatYouTubeVideosAreDifferent = new Random();
      Vector3D randomCameraOffset = RandomGeometry.nextVector3D(randomForSlightlyMovingCameraSoThatYouTubeVideosAreDifferent, 0.05);
      cameraFix.add(randomCameraOffset);

      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(false, true, true, false);
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
}
