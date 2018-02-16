package us.ihmc.avatar.simulationStarter;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.avatar.DRCLidar;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.VideoDataServerImageCallback;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.producers.RawVideoDataServer;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotDataVisualizer.logger.BehaviorVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.SCSPlaybackListener;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;

public class DRCSimulationStarter implements SimulationStarterInterface
{
   private static final boolean DEBUG = false;

   private final DRCRobotModel robotModel;
   private final CommonAvatarEnvironmentInterface environment;
   private final DRCSCSInitialSetup scsInitialSetup;

   private DRCGuiInitialSetup guiInitialSetup;
   private DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;

   private HumanoidFloatingRootJointRobot sdfRobot;
   private HighLevelHumanoidControllerFactory controllerFactory;
   private AvatarSimulation avatarSimulation;
   private SimulationConstructionSet simulationConstructionSet;

   private ScriptBasedControllerCommandGenerator scriptBasedControllerCommandGenerator;
   private boolean createSCSSimulatedSensors;

   private boolean addFootstepMessageGenerator = false;
   private boolean useHeadingAndVelocityScript = false;
   private boolean cheatWithGroundHeightAtForFootstep = false;

   private PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber;
   private HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters;

   /**
    * The PacketCommunicator used as input of the controller is either equal to the output PacketCommunicator of the network processor or the behavior module if any.
    * It is bidirectional meaning that it carries commands to be executed by the controller and that the controller is able to send feedback the other way to whoever is listening to the PacketCommunicator.
    */
   private PacketCommunicator controllerPacketCommunicator;

   /** The output PacketCommunicator of the simulation carries sensor information (LIDAR, camera, etc.) and is used as input of the network processor. */
   private LocalObjectCommunicator scsSensorOutputPacketCommunicator;

   private boolean setupControllerNetworkSubscriber = true;

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private final JumpControllerParameters jumpControlParameters;
   private final RobotContactPointParameters contactPointParameters;

   private final Point3D scsCameraPosition = new Point3D(6.0, -2.0, 4.5);
   private final Point3D scsCameraFix = new Point3D(-0.44, -0.17, 0.75);

   private final List<HighLevelControllerStateFactory> highLevelControllerFactories = new ArrayList<>();
   private final List<ControllerStateTransitionFactory<HighLevelControllerName>> controllerTransitionFactories = new ArrayList<>();
   private DRCNetworkProcessor networkProcessor;

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();

   private final ConcurrentLinkedQueue<Command<?, ?>> controllerCommands = new ConcurrentLinkedQueue<>();

   public DRCSimulationStarter(DRCRobotModel robotModel, GroundProfile3D groundProfile3D)
   {
      this(robotModel, null, groundProfile3D);
   }

   public DRCSimulationStarter(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      this(robotModel, environment, environment.getTerrainObject3D());
   }

   private DRCSimulationStarter(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment, GroundProfile3D groundProfile3D)
   {
      this.robotModel = robotModel;
      this.environment = environment;

      this.guiInitialSetup = new DRCGuiInitialSetup(false, false);
      this.robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);

      this.createSCSSimulatedSensors = true;

      this.scsInitialSetup = new DRCSCSInitialSetup(groundProfile3D, robotModel.getSimulateDT());
      this.scsInitialSetup.setDrawGroundProfile(environment == null);
      this.scsInitialSetup.setInitializeEstimatorToActual(false);
      this.scsInitialSetup.setTimePerRecordTick(robotModel.getControllerDT());
      this.scsInitialSetup.setRunMultiThreaded(true);

      this.highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      this.walkingControllerParameters = robotModel.getWalkingControllerParameters();
      this.capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      this.jumpControlParameters = robotModel.getJumpControlParameters();
      this.contactPointParameters = robotModel.getContactPointParameters();
   }

   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      return environment;
   }

   /**
    * Register a controller to be created in addition to the walking controller.
    * For instance, the {@link CarIngressEgressController} can be created by passing its factory, i.e. {@link CarIngressEgressControllerFactory}.
    * The active controller can then be switched by either changing the variable {@code requestedHighLevelState} from SCS or by sending a {@link HighLevelStateMessage} to the controller.
    * @param controllerFactory a factory to create an additional controller.
    */
   public void registerHighLevelControllerState(HighLevelControllerStateFactory controllerFactory)
   {
      this.highLevelControllerFactories.add(controllerFactory);
   }

   public void registerControllerStateTransition(ControllerStateTransitionFactory<HighLevelControllerName> controllerStateTransitionFactory)
   {
      this.controllerTransitionFactories.add(controllerStateTransitionFactory);
   }

   /**
    * Call this method to disable simulated sensors such as LIDAR and camera.
    */
   public void disableSCSSimulatedSensors()
   {
      this.createSCSSimulatedSensors = false;
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      if (controllerFactory == null)
         this.controllerFailureListeners.add(listener);
      else
         controllerFactory.attachControllerFailureListener(listener);
   }

   /**
    * Returns the SRCSCSInitialSetup. Can use that object to directly change things in the sim. But need to make those changes before calling createAvatarSimulation().
    * @return SRCSCSInitialSetup
    */
   public DRCSCSInitialSetup getSCSInitialSetup()
   {
      return scsInitialSetup;
   }

   /**
    * Sets whether the estimator and the controller are running on the same thread or multiThreaded. Defaults to multiThreaded.
    * Need to set to false if you want the simulation to be rewindable.
    * @param runMultiThreaded
    */
   public void setRunMultiThreaded(boolean runMultiThreaded)
   {
      scsInitialSetup.setRunMultiThreaded(runMultiThreaded);
   }

   public void setupControllerNetworkSubscriber(boolean setup)
   {
      setupControllerNetworkSubscriber = setup;
   }

   /**
    * Set whether state estimation as on the the real robot or perfect sensors coming from the simulated robot should be used.
    * By default the state estimator is used.
    * @param usePerfectSensors
    */
   public void setUsePerfectSensors(boolean usePerfectSensors)
   {
      scsInitialSetup.setUsePerfectSensors(usePerfectSensors);
   }

   /**
    * Indicates if the state estimator should be aware of the robot starting location.
    * It is set to false by default, meaning that independently from the robot starting location, the state estimator will think that the robot started at (0, 0) in world but will be aware of the initial yaw.
    */
   public void setInitializeEstimatorToActual(boolean initializeEstimatorToActual)
   {
      scsInitialSetup.setInitializeEstimatorToActual(initializeEstimatorToActual);
   }

   /**
    * Provide a subscriber for receiving pelvis poses (for instance from the iterative closest point module) to be accounted for in the state estimator.
    * @param externalPelvisCorrectorSubscriber
    */
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber)
   {
      if (avatarSimulation != null)
         avatarSimulation.setExternalPelvisCorrectorSubscriber(externalPelvisCorrectorSubscriber);
      else
         this.externalPelvisCorrectorSubscriber = externalPelvisCorrectorSubscriber;
   }

   /**
    * Set a GUI initial setup. If not called, a default GUI initial setup is used.
    */
   public void setGuiInitialSetup(DRCGuiInitialSetup guiInitialSetup)
   {
      this.guiInitialSetup = guiInitialSetup;
   }

   /**
    * Set a robot initial setup to use instead of the one in DRCRobotModel.
    * @param robotInitialSetup
    */
   public void setRobotInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   /**
    * Set a specific starting location. By default, the robot will start at (0, 0) in world with no yaw.
    */
   @Override
   public void setStartingLocation(DRCStartingLocation startingLocation)
   {
      setStartingLocationOffset(startingLocation.getStartingLocationOffset());
   }

   /**
    * Set a specific starting location offset. By default, the robot will start at (0, 0) in world with no yaw.
    */
   public void setStartingLocationOffset(OffsetAndYawRobotInitialSetup startingLocationOffset)
   {
      robotInitialSetup.setInitialYaw(startingLocationOffset.getYaw());
      robotInitialSetup.setInitialGroundHeight(startingLocationOffset.getGroundHeight());
      robotInitialSetup.setOffset(startingLocationOffset.getAdditionalOffset());
   }

   /**
    * Set a specific starting location offset. By default, the robot will start at (0, 0) in world with no yaw.
    */
   public void setStartingLocationOffset(Vector3D robotInitialPosition, double yaw)
   {
      setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(robotInitialPosition, yaw));
   }

   /**
    * Sets the initial SCS camera position.
    * @param positionX
    * @param positionY
    * @param positionZ
    */
   public void setSCSCameraPosition(double positionX, double positionY, double positionZ)
   {
      scsCameraPosition.set(positionX, positionY, positionZ);
   }

   /**
    * Sets the initial fix point that the SCS camera looks at.
    * @param fixX
    * @param fixY
    * @param fixZ
    */
   public void setSCSCameraFix(double fixX, double fixY, double fixZ)
   {
      scsCameraFix.set(fixX, fixY, fixZ);
   }

   //   /** Make the controller use a specific PacketCommunicator instead of the default. If you don't know what you're doing, forget about that method. */
   //   public void setControllerPacketCommunicator(PacketCommunicator controllerInputPacketCommunicator)
   //   {
   //      this.controllerPacketCommunicator = controllerInputPacketCommunicator;
   //   }

   /**
    * Creates a default output PacketCommunicator for the network processor.
    * This PacketCommunicator is also set to be used as input for the controller.
    * @param networkParameters
    */
   private void createControllerCommunicator(DRCNetworkModuleParameters networkParameters)
   {
      // Don't do anything if the network processor has already been setup
      if (controllerPacketCommunicator != null)
         return;

      networkParameters.enableLocalControllerCommunicator(true);

      controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                             new IHMCCommunicationKryoNetClassList());
      try
      {
         controllerPacketCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Starts the SCS visualizer for the behavior module.
    */
   @Override
   public void startBehaviorVisualizer()
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);
      spawner.spawn(BehaviorVisualizer.class);
   }

   /**
    * Creates and starts the simulation and automatically starts the network processor if required.
    * All the specific requirements (environment, robot initial setup, etc.) have to be set before calling this method.
    * @param startNetworkProcessor if true the network processor is created and started.
    * @param automaticallyStartSimulation if true SCS will be simulating when it shows up.
    * @return
    */
   @Override
   public void startSimulation(DRCNetworkModuleParameters networkParameters, boolean automaticallySimulate)
   {
      createSimulation(networkParameters, true, automaticallySimulate);
   }

   public void createSimulation(DRCNetworkModuleParameters networkParameters, boolean automaticallySpawnSimulation, boolean automaticallySimulate)
   {
      if ((networkParameters != null))
      {
         createControllerCommunicator(networkParameters);
      }

      this.avatarSimulation = createAvatarSimulation();

      if (automaticallySpawnSimulation)
         avatarSimulation.start();

      if (automaticallySpawnSimulation && automaticallySimulate)
         avatarSimulation.simulate();

      if ((networkParameters != null) && networkParameters.isNetworkProcessorEnabled()) //&& (networkParameters.useController()))
      {
         startNetworkProcessor(networkParameters);
      }
   }

   public ScriptBasedControllerCommandGenerator getScriptBasedControllerCommandGenerator()
   {
      return scriptBasedControllerCommandGenerator;
   }

   public void setFlatGroundWalkingScriptParameters(HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      this.walkingScriptParameters = walkingScriptParameters;
   }

   private AvatarSimulation createAvatarSimulation()
   {
      HumanoidGlobalDataProducer dataProducer = null;
      if (controllerPacketCommunicator != null)
      {
         dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicator);
      }

      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames, feetContactSensorNames, wristForceSensorNames,
                                                                 highLevelControllerParameters, walkingControllerParameters, capturePointPlannerParameters, jumpControlParameters);
      setupHighLevelStates(controllerFactory);

      controllerFactory.attachControllerFailureListeners(controllerFailureListeners);
      if (setupControllerNetworkSubscriber)
         controllerFactory.createControllerNetworkSubscriber(new PeriodicNonRealtimeThreadScheduler("CapturabilityBasedStatusProducer"),
                                                             controllerPacketCommunicator);

      for (int i = 0; i < highLevelControllerFactories.size(); i++)
         controllerFactory.addCustomControlState(highLevelControllerFactories.get(i));
      for (int i = 0; i < controllerTransitionFactories.size(); i++)
         controllerFactory.addCustomStateTransition(controllerTransitionFactories.get(i));

      controllerFactory.setInitialState(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      controllerFactory.createQueuedControllerCommandGenerator(controllerCommands);

      controllerFactory.setHeadingAndVelocityEvaluationScriptParameters(walkingScriptParameters);

      if (addFootstepMessageGenerator && cheatWithGroundHeightAtForFootstep)
         controllerFactory.createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, scsInitialSetup.getHeightMap());
      else if (addFootstepMessageGenerator)
         controllerFactory.createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript);

      AvatarSimulationFactory avatarSimulationFactory = new AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvironment(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setHumanoidGlobalDataProducer(dataProducer);
      AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      HighLevelHumanoidControllerToolbox highLevelHumanoidControllerToolbox = controllerFactory.getHighLevelHumanoidControllerToolbox();
      FullHumanoidRobotModel fullRobotModel = highLevelHumanoidControllerToolbox.getFullRobotModel();
      scriptBasedControllerCommandGenerator = new ScriptBasedControllerCommandGenerator(controllerCommands, fullRobotModel);

      if (externalPelvisCorrectorSubscriber != null)
         avatarSimulation.setExternalPelvisCorrectorSubscriber(externalPelvisCorrectorSubscriber);

      simulationConstructionSet = avatarSimulation.getSimulationConstructionSet();
      sdfRobot = avatarSimulation.getHumanoidFloatingRootJointRobot();

      simulationConstructionSet.setCameraPosition(scsCameraPosition.getX(), scsCameraPosition.getY(), scsCameraPosition.getZ());
      simulationConstructionSet.setCameraFix(scsCameraFix.getX(), scsCameraFix.getY(), scsCameraFix.getZ());

      PlaybackListener playbackListener = new SCSPlaybackListener(dataProducer);
      simulationConstructionSet.attachPlaybackListener(playbackListener);

      return avatarSimulation;
   }

   public void setupHighLevelStates(HighLevelHumanoidControllerFactory controllerFactory)
   {
      controllerFactory.useDefaultDoNothingControlState();
      //controllerFactory.useDefaultWalkingControlState();
      if(jumpControlParameters != null)
      {
         controllerFactory.useDefaultJumpControllerState();
      }

      //controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      //controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);
      
      if(jumpControlParameters != null)
      {
         controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, JUMPING);
         controllerFactory.addRequestableTransition(JUMPING, DO_NOTHING_BEHAVIOR);
      }
   }

   public ConcurrentLinkedQueue<Command<?, ?>> getQueuedControllerCommands()
   {
      return controllerCommands;
   }

   public LocalObjectCommunicator createSimulatedSensorsPacketCommunicator()
   {
      scsSensorOutputPacketCommunicator = new LocalObjectCommunicator();

      if (createSCSSimulatedSensors)
      {
         DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         DRCRobotJointMap jointMap = robotModel.getJointMap();
         TimestampProvider timeStampProvider = avatarSimulation.getSimulatedRobotTimeProvider();
         HumanoidFloatingRootJointRobot robot = avatarSimulation.getHumanoidFloatingRootJointRobot();
         Graphics3DAdapter graphics3dAdapter = simulationConstructionSet.getGraphics3dAdapter();

         printIfDebug("Streaming SCS Video");
         DRCRobotCameraParameters cameraParameters = sensorInformation.getCameraParameters(0);
         if (cameraParameters != null)
         {
            String cameraName = cameraParameters.getSensorNameInSdf();
            int width = sdfRobot.getCameraMount(cameraName).getImageWidth();
            int height = sdfRobot.getCameraMount(cameraName).getImageHeight();

            CameraConfiguration cameraConfiguration = new CameraConfiguration(cameraName);
            cameraConfiguration.setCameraMount(cameraName);

            int framesPerSecond = 25;
            RawVideoDataServer drcRenderedSceneVideoHandler = new RawVideoDataServer(scsSensorOutputPacketCommunicator);
            simulationConstructionSet.startStreamingVideoData(cameraConfiguration, width, height, new VideoDataServerImageCallback(drcRenderedSceneVideoHandler), timeStampProvider,
                                                              framesPerSecond);
         }

         for (DRCRobotLidarParameters lidarParams : sensorInformation.getLidarParameters())
         {
            DRCLidar.setupDRCRobotLidar(robot, graphics3dAdapter, scsSensorOutputPacketCommunicator, jointMap, lidarParams, timeStampProvider, true);
         }
      }

      return scsSensorOutputPacketCommunicator;
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   /**
    * Creates and starts the network processor.
    * The network processor is necessary to run the behavior module and/or the operator interface.
    * It has to be created after the simulation.
    * @param networkModuleParams
    */
   private void startNetworkProcessor(DRCNetworkModuleParameters networkModuleParams)
   {
      if (networkModuleParams.isRosModuleEnabled() || networkModuleParams.isSensorModuleEnabled())
      {
         LocalObjectCommunicator simulatedSensorCommunicator = createSimulatedSensorsPacketCommunicator();
         networkModuleParams.setSimulatedSensorCommunicator(simulatedSensorCommunicator);
      }

      networkProcessor = new DRCNetworkProcessor(robotModel, networkModuleParams);
   }

   public AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public HumanoidFloatingRootJointRobot getSDFRobot()
   {
      return sdfRobot;
   }

   public PacketRouter<PacketDestination> getPacketRouter()
   {
      return networkProcessor.getPacketRouter();
   }

   public LocalObjectCommunicator getSimulatedSensorsPacketCommunicator()
   {
      if (scsSensorOutputPacketCommunicator == null)
      {
         createSimulatedSensorsPacketCommunicator();
      }
      return scsSensorOutputPacketCommunicator;
   }

   @Override
   public void close()
   {
      if (controllerPacketCommunicator != null)
      {
         controllerPacketCommunicator.disconnect();
      }
   }

   public void addFootstepMessageGenerator(boolean useHeadingAndVelocityScript, boolean cheatWithGroundHeightAtForFootstep)
   {
      addFootstepMessageGenerator = true;
      this.useHeadingAndVelocityScript = useHeadingAndVelocityScript;
      this.cheatWithGroundHeightAtForFootstep = cheatWithGroundHeightAtForFootstep;
   }
}
