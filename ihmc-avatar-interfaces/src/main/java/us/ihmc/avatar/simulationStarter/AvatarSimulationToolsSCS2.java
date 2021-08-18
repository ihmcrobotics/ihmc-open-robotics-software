package us.ihmc.avatar.simulationStarter;

import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.createNetworkProcessorParameters;
import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.showSelectorWithStartingLocation;
import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.startOpertorInterface;
import static us.ihmc.avatar.simulationStarter.DRCSimulationTools.startOpertorInterfaceUsingProcessSpawner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools.Modules;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataVisualizer.logger.BehaviorVisualizer;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.RemoteLidarBasedREAUILauncher;
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
         avatarSimulationEnvironment = new AvatarSimulationEnvironment();
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
            avatarSimulationEnvironment = new AvatarSimulationEnvironment();
         HumanoidNetworkProcessorParameters networkModuleParams = createNetworkProcessorParameters(modulesToStart);

         if (networkModuleParams.isUseROSModule() || networkModuleParams.isUseSensorModule())
         {
            // FIXME Can't be setup with SCS2 right now :/
            //            LocalObjectCommunicator simulatedSensorCommunicator = createSimulatedSensorsPacketCommunicator();
            //            networkModuleParams.setSimulatedSensorCommunicator(simulatedSensorCommunicator);
         }

         avatarSimulationEnvironment.networkProcessor = HumanoidNetworkProcessor.newFromParameters(robotModel,
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   networkModuleParams);
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
      private SCS2AvatarSimulationFactory avatarSimulationFactory;
      private SCS2AvatarSimulation avatarSimulation;
      private HumanoidNetworkProcessor networkProcessor;

      public void createSimulation()
      {
         if (avatarSimulationFactory != null)
            avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
      }

      public void start()
      {
         if (avatarSimulation != null)
            avatarSimulation.start();
         if (networkProcessor != null)
            networkProcessor.start();
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
