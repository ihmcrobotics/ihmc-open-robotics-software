package us.ihmc.darpaRoboticsChallenge;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import com.github.quickhull3d.Point3d;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.CarIngressEgressControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DataProducerVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.YoVariableVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CarIngressEgressController;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.scriptEngine.VariousWalkingProviderFromScriptFactory;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCRenderedSceneVideoHandler;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.humanoidBehaviors.BehaviorVisualizer;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.GlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class DRCSimulationStarter
{
   private static final boolean DEBUG = false;

   private final DRCRobotModel robotModel;
   private final CommonAvatarEnvironmentInterface environment;
   private final DRCSCSInitialSetup scsInitialSetup;

   private DRCGuiInitialSetup guiInitialSetup;
   private DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup;

   private SDFHumanoidRobot sdfRobot;
   private MomentumBasedControllerFactory controllerFactory;
   private DRCSimulationFactory drcSimulationFactory;
   private SimulationConstructionSet simulationConstructionSet;

   private String scriptFileName;
   private boolean createSCSSimulatedSensors;

   private boolean deactivateWalkingFallDetector = false;
   
   private PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber;

   /**
    * The PacketCommunicator used as input of the controller is either equal to the output PacketCommunicator of the network processor or the behavior module if any.
    * It is bidirectional meaning that it carries commands to be executed by the controller and that the controller is able to send feedback the other way to whoever is listening to the PacketCommunicator.
    */
   private PacketCommunicator controllerPacketCommunicator;

   /** The output PacketCommunicator of the simulation carries sensor information (LIDAR, camera, etc.) and is used as input of the network processor. */
   private LocalObjectCommunicator scsSensorOutputPacketCommunicator;

   private final WalkingControllerParameters walkingControllerParameters;
   private final ArmControllerParameters armControllerParameters;
   private final CapturePointPlannerParameters capturePointPlannerParameters;
   private final RobotContactPointParameters contactPointParameters;

   private final Point3d scsCameraPosition = new Point3d(6.0, -2.0, 4.5);
   private final Point3d scsCameraFix = new Point3d(-0.44, -0.17, 0.75);

   private final List<HighLevelBehaviorFactory> highLevelBehaviorFactories = new ArrayList<>();
   private DRCNetworkProcessor networkProcessor;

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();

   public DRCSimulationStarter(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      this.robotModel = robotModel;
      this.environment = environment;
      
      this.guiInitialSetup = new DRCGuiInitialSetup(false, false);
      this.robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);

      this.createSCSSimulatedSensors = true;
      
      scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(false);
      scsInitialSetup.setTimePerRecordTick(robotModel.getControllerDT());
      scsInitialSetup.setRunMultiThreaded(true);

      this.walkingControllerParameters = robotModel.getWalkingControllerParameters();
      this.armControllerParameters = robotModel.getArmControllerParameters();
      this.capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      this.contactPointParameters = robotModel.getContactPointParameters();
   }

   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      return environment;
   }

   /**
    * Deativate the controller failure detector.
    * If called the walking controller will running even if the robot falls.
    */
   public void deactivateWalkingFallDetector()
   {
      deactivateWalkingFallDetector = true;
   }

   /**
    * Register a controller to be created in addition to the walking controller.
    * For instance, the {@link CarIngressEgressController} can be created by passing its factory, i.e. {@link CarIngressEgressControllerFactory}.
    * The active controller can then be switched by either changing the variable {@code requestedHighLevelState} from SCS or by sending a {@link HighLevelStatePacket} to the controller.
    * @param controllerFactory a factory to create an additional controller.
    */
   public void registerHighLevelController(HighLevelBehaviorFactory controllerFactory)
   {
      this.highLevelBehaviorFactories.add(controllerFactory);
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
    * Mostly used for unit tests.
    * Set the file name of a script to be loaded and executed by the controller.
    * @param scriptFileName
    */
   public void setScriptFile(String scriptFileName)
   {
      this.scriptFileName = scriptFileName;
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
      if (drcSimulationFactory != null)
         drcSimulationFactory.setExternalPelvisCorrectorSubscriber(externalPelvisCorrectorSubscriber);
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
   public void setRobotInitialSetup(DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   /**
    * Set a specific starting location. By default, the robot will start at (0, 0) in world with no yaw.
    */
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
   public void setStartingLocationOffset(Vector3d robotInitialPosition, double yaw)
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
      
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, netClassList);
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
   public void startBehaviorVisualizer()
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true);
      spawner.spawn(BehaviorVisualizer.class);
   }

   /**
    * Creates and starts the simulation and automatically starts the network processor if required.
    * All the specific requirements (environment, robot initial setup, etc.) have to be set before calling this method.
    * @param startNetworkProcessor if true the network processor is created and started.
    * @param automaticallyStartSimulation if true SCS will be simulating when it shows up.
    * @return
    */
   public void startSimulation(DRCNetworkModuleParameters networkParameters, boolean automaticallyStartSimulation)
   {
      if ((networkParameters != null)) // && (networkParameters.useController())) 
      {
         createControllerCommunicator(networkParameters);
      }
      
      createSimulationFactory();

      if (automaticallyStartSimulation)
         drcSimulationFactory.simulate();

      if ((networkParameters != null) && networkParameters.isNetworkProcessorEnabled()) //&& (networkParameters.useController()))
      {
         startNetworkProcessor(networkParameters);
      }
   }

   private void createSimulationFactory()
   {
      GlobalDataProducer dataProducer = null;
      if (controllerPacketCommunicator != null)
      {
         dataProducer = new GlobalDataProducer(controllerPacketCommunicator);
      }

      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames, feetContactSensorNames, wristForceSensorNames,
            walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, HighLevelState.WALKING);
      controllerFactory.attachControllerFailureListeners(controllerFailureListeners);
      
      for (int i = 0; i < highLevelBehaviorFactories.size(); i++)
         controllerFactory.addHighLevelBehaviorFactory(highLevelBehaviorFactories.get(i));

      if (deactivateWalkingFallDetector)
         controllerFactory.setFallbackControllerForFailure(null);

      registerWalkingProviderFactory(dataProducer, controllerFactory);

      drcSimulationFactory = new DRCSimulationFactory(robotModel, controllerFactory, environment, robotInitialSetup, scsInitialSetup, guiInitialSetup, dataProducer);

      if (externalPelvisCorrectorSubscriber != null)
         drcSimulationFactory.setExternalPelvisCorrectorSubscriber(externalPelvisCorrectorSubscriber);

      simulationConstructionSet = drcSimulationFactory.getSimulationConstructionSet();
      sdfRobot = drcSimulationFactory.getRobot();

      simulationConstructionSet.setCameraPosition(scsCameraPosition.x, scsCameraPosition.y, scsCameraPosition.z);
      simulationConstructionSet.setCameraFix(scsCameraFix.x, scsCameraFix.y, scsCameraFix.z);

      PlaybackListener playbackListener = new SCSPlaybackListener(dataProducer);
      simulationConstructionSet.attachPlaybackListener(playbackListener);

      drcSimulationFactory.start();
   }

   private void registerWalkingProviderFactory(GlobalDataProducer dataProducer, MomentumBasedControllerFactory controllerFactory)
   {
      if ((scriptFileName != null) && (!scriptFileName.equals("")))
      {
         // Create providers that read a script file with command to be executed by the controller.
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptFileName);
         VariousWalkingProviderFromScriptFactory variousWalkingProviderFactory = new VariousWalkingProviderFromScriptFactory(scriptInputStream);
         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      }
      else if (dataProducer != null)
      {
         // Create providers that listen to incoming packets through the GlobalDataProducer, some of the producers are also able to send feedback.
         FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(walkingControllerParameters);
         VariousWalkingProviderFactory variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters, new PeriodicNonRealtimeThreadScheduler("CapturabilityBasedStatusProducer"));
         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      }
      else
      {
         // Create providers that use YoVariables instead of packets or scripts. Mostly used for debugging or when the operator interface is inaccessible.
         YoVariableVariousWalkingProviderFactory variousWalkingProviderFactory = new YoVariableVariousWalkingProviderFactory();
         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      }
   }

   public LocalObjectCommunicator createSimulatedSensorsPacketCommunicator()
   {
      scsSensorOutputPacketCommunicator = new LocalObjectCommunicator();

      if (createSCSSimulatedSensors)
      {
         DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         DRCRobotJointMap jointMap = robotModel.getJointMap();
         TimestampProvider timeStampProvider = drcSimulationFactory.getTimeStampProvider();

         printIfDebug("Streaming SCS Video");
         DRCRobotCameraParameters cameraParameters = sensorInformation.getCameraParameters(0);
         if (cameraParameters != null)
         {
            String cameraName = cameraParameters.getSensorNameInSdf();
            int width = sdfRobot.getCamera(cameraName).getWidth();
            int height = sdfRobot.getCamera(cameraName).getHeight();

            CameraConfiguration cameraConfiguration = new CameraConfiguration(cameraName);
            cameraConfiguration.setCameraMount(cameraName);

            int framesPerSecond = 25;
            DRCRenderedSceneVideoHandler drcRenderedSceneVideoHandler = new DRCRenderedSceneVideoHandler(scsSensorOutputPacketCommunicator);
            simulationConstructionSet.startStreamingVideoData(cameraConfiguration, width, height, drcRenderedSceneVideoHandler, timeStampProvider, framesPerSecond);
         }

         for (DRCRobotLidarParameters lidarParams : sensorInformation.getLidarParameters())
         {
            DRCLidar.setupDRCRobotLidar(drcSimulationFactory, scsSensorOutputPacketCommunicator, jointMap, lidarParams, timeStampProvider, true);
         }
      }

      return scsSensorOutputPacketCommunicator;
   }

   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }

   /**
    * Creates and starts the network processor.
    * The network processor is necessary to run the behavior module and/or the operator interface.
    * It has to be created after the simulation.
    * @param networkModuleParams 
    */
   private void startNetworkProcessor(DRCNetworkModuleParameters networkModuleParams)
   {
      if(networkModuleParams.isRosModuleEnabled() || networkModuleParams.isSensorModuleEnabled())
      {
         LocalObjectCommunicator simulatedSensorCommunicator = createSimulatedSensorsPacketCommunicator();
         networkModuleParams.setSimulatedSensorCommunicator(simulatedSensorCommunicator);
      }
      
      networkProcessor = new DRCNetworkProcessor(robotModel, networkModuleParams);
   }

   public DRCSimulationFactory getDRCSimulationFactory()
   {
      return drcSimulationFactory;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public SDFHumanoidRobot getSDFRobot()
   {
      return sdfRobot;
   }

   public PacketRouter<PacketDestination> getPacketRouter()
   {
      return networkProcessor.getPacketRouter();
   }

   public LocalObjectCommunicator getSimulatedSensorsPacketCommunicator()
   {
      if(scsSensorOutputPacketCommunicator == null)
      {
         createSimulatedSensorsPacketCommunicator();
      }
      return scsSensorOutputPacketCommunicator;
   }

   public void close()
   {
      if(controllerPacketCommunicator != null)
      {
         controllerPacketCommunicator.close();
      }
   }
}
