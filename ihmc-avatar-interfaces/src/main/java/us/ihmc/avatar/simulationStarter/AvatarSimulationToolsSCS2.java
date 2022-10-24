package us.ihmc.avatar.simulationStarter;

import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.createNetworkProcessorParameters;
import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.showSelectorWithStartingLocation;
import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.startOpertorInterface;
import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.startOpertorInterfaceUsingProcessSpawner;

import java.util.ArrayList;
import java.util.List;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools.Modules;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataVisualizer.logger.BehaviorVisualizer;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.RemoteLidarBasedREAUILauncher;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.sensors.SimCameraSensor;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public class AvatarSimulationToolsSCS2
{
   @SuppressWarnings({"hiding", "unchecked"})
   public static <T extends DRCStartingLocation, Enum> AvatarSimulationEnvironment setupSimulationEnvironmentWithGraphicSelector(DRCRobotModel robotModel,
                                                                                                                                 CommonAvatarEnvironmentInterface environment,
                                                                                                                                 Class<?> operatorInterfaceClass,
                                                                                                                                 String[] operatorInterfaceArgs,
                                                                                                                                 T... possibleStartingLocations)
   {
      List<Modules> modulesToStart = new ArrayList<>();
      DRCStartingLocation startingLocation = showSelectorWithStartingLocation(modulesToStart, possibleStartingLocations);

      return setupSimulationEnvironment(robotModel, environment, operatorInterfaceClass, operatorInterfaceArgs, startingLocation, modulesToStart);
   }

   @SuppressWarnings({"hiding"})
   public static <T extends DRCStartingLocation, Enum> AvatarSimulationEnvironment setupSimulationEnvironment(DRCRobotModel robotModel,
                                                                                                              CommonAvatarEnvironmentInterface environment,
                                                                                                              Class<?> operatorInterfaceClass,
                                                                                                              String[] operatorInterfaceArgs,
                                                                                                              DRCStartingLocation startingLocation,
                                                                                                              List<Modules> modulesToStart)
   {
      AvatarSimulationEnvironment avatarSimulationEnvironment = null;

      if (modulesToStart.isEmpty())
         return null;

      if (modulesToStart.contains(Modules.SIMULATION))
      {
         avatarSimulationEnvironment = new AvatarSimulationEnvironment(robotModel);
         boolean automaticallyStartSimulation = true;
         avatarSimulationEnvironment.avatarSimulationFactory = new SCS2AvatarSimulationFactory();
         avatarSimulationEnvironment.avatarSimulationFactory.setRobotModel(robotModel);
         avatarSimulationEnvironment.avatarSimulationFactory.setRealtimeROS2Node(ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS,
                                                                                                                  "ihmc_simulation"));
         avatarSimulationEnvironment.avatarSimulationFactory.setDefaultHighLevelHumanoidControllerFactory();
         avatarSimulationEnvironment.avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(environment);
         avatarSimulationEnvironment.avatarSimulationFactory.setSimulationDataRecordTimePeriod(robotModel.getControllerDT());
         avatarSimulationEnvironment.avatarSimulationFactory.setStartingLocationOffset(startingLocation.getStartingLocationOffset());
         avatarSimulationEnvironment.avatarSimulationFactory.setAutomaticallyStartSimulation(automaticallyStartSimulation);
      }

      if (modulesToStart.contains(Modules.NETWORK_PROCESSOR))
      {
         if (avatarSimulationEnvironment == null)
            avatarSimulationEnvironment = new AvatarSimulationEnvironment(robotModel);
         avatarSimulationEnvironment.networkProcessorParameters = createNetworkProcessorParameters(modulesToStart);
      }

      if (modulesToStart.contains(Modules.OPERATOR_INTERFACE))
      {
         if (modulesToStart.contains(Modules.SIMULATION))
            startOpertorInterfaceUsingProcessSpawner(operatorInterfaceClass, operatorInterfaceArgs);
         else
            startOpertorInterface(operatorInterfaceClass, operatorInterfaceArgs);
      }

      if (modulesToStart.contains(Modules.BEHAVIOR_VISUALIZER))
      {
         JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);
         spawner.spawn(BehaviorVisualizer.class);
      }

      boolean startREAModule = modulesToStart.contains(Modules.REA_MODULE);
      boolean startREAUI = modulesToStart.contains(Modules.REA_UI);

      if (startREAModule && startREAUI)
         new JavaProcessSpawner(true, true).spawn(LidarBasedREAStandaloneLauncher.class);
      else if (startREAUI)
         new JavaProcessSpawner(true, true).spawn(RemoteLidarBasedREAUILauncher.class);

      return avatarSimulationEnvironment;
   }

   public static class AvatarSimulationEnvironment
   {
      private final DRCRobotModel robotModel;

      private SCS2AvatarSimulationFactory avatarSimulationFactory;
      private HumanoidNetworkProcessorParameters networkProcessorParameters;

      private SCS2AvatarSimulation avatarSimulation;
      private HumanoidNetworkProcessor networkProcessor;

      public AvatarSimulationEnvironment(DRCRobotModel robotModel)
      {
         this.robotModel = robotModel;
      }

      public void build()
      {
         if (avatarSimulationFactory == null)
            return;

         avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

         if (networkProcessorParameters != null)
         {
            if (networkProcessorParameters.isUseROSModule() || networkProcessorParameters.isUseSensorModule())
            {
               networkProcessorParameters.setSimulatedSensorCommunicator(createSimulatedSensorsPacketCommunicator());
            }

            networkProcessor = HumanoidNetworkProcessor.newFromParameters(robotModel, PubSubImplementation.FAST_RTPS, networkProcessorParameters);
         }
      }

      private LocalObjectCommunicator createSimulatedSensorsPacketCommunicator()
      {
         LocalObjectCommunicator simulatedSensorCommunicator = new LocalObjectCommunicator();

         HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         Robot robot = avatarSimulation.getRobot();

         AvatarRobotCameraParameters cameraParameters = sensorInformation.getCameraParameters(0);

         if (cameraParameters != null)
         {
            String cameraName = cameraParameters.getSensorNameInSdf();

            SimCameraSensor cameraSensor = robot.getAllJoints().stream().flatMap(joint -> joint.getAuxialiryData().getCameraSensors().stream())
                                                .filter(camera -> camera.getName().equals(cameraName)).findFirst().get();

            cameraSensor.setEnable(true);

            int width = cameraSensor.getImageWidth().getValue();
            int height = cameraSensor.getImageHeight().getValue();
            double fov = cameraSensor.getFieldOfView().getValue();

            double f = width / 2 / Math.tan(fov / 2);
            CameraPinholeBrown intrinsicParameters = new CameraPinholeBrown(f, f, 0, (width - 1) / 2f, (height - 1) / 2f, width, height);

            cameraSensor.addCameraFrameConsumer((timestamp, frame) ->
            {
               LocalVideoPacket videoPacket = HumanoidMessageTools.createLocalVideoPacket(timestamp, frame, intrinsicParameters);
               simulatedSensorCommunicator.consumeObject(videoPacket);
            });
         }

         if (sensorInformation.getLidarParameters() != null)
         {
            // TODO Implement me!
         }

         return simulatedSensorCommunicator;
      }

      public void start()
      {
         if (avatarSimulation != null)
            avatarSimulation.start();
         if (networkProcessor != null)
         {
            networkProcessor.start();
            if (avatarSimulation.getSimulationConstructionSet() != null)
               avatarSimulation.getSimulationConstructionSet().addVisualizerShutdownListener(() -> networkProcessor.closeAndDispose());
         }
      }

      public SCS2AvatarSimulationFactory getAvatarSimulationFactory()
      {
         return avatarSimulationFactory;
      }

      public SCS2AvatarSimulation getAvatarSimulation()
      {
         return avatarSimulation;
      }

      public HumanoidNetworkProcessor getNetworkProcessor()
      {
         return networkProcessor;
      }
   }
}
