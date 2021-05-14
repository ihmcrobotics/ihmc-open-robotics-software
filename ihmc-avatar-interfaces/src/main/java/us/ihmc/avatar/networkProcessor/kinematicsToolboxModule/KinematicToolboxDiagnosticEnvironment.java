package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class KinematicToolboxDiagnosticEnvironment
{
   private final String threadName = "NonRealtimeScheduler";
   private final RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "ihmc_fake_controller");

   public KinematicToolboxDiagnosticEnvironment(DRCRobotModel drcRobotModel)
   {
      FullHumanoidRobotModel humanoidFullRobotModel = drcRobotModel.createFullRobotModel();
      HumanoidJointNameMap jointMap = drcRobotModel.getJointMap();
      HumanoidFloatingRootJointRobot humanoidFloatingRobotModel = drcRobotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = drcRobotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      robotInitialSetup.initializeRobot(humanoidFloatingRobotModel, jointMap);
      SDFPerfectSimulatedSensorReader sdfPerfectReader = new SDFPerfectSimulatedSensorReader(humanoidFloatingRobotModel, humanoidFullRobotModel, null);
      sdfPerfectReader.read();

      ForceSensorDefinition[] forceSensorDefinitionArray = humanoidFullRobotModel.getForceSensorDefinitions();
      List<ForceSensorDefinition> forceSensorDefinitionList = Arrays.asList(forceSensorDefinitionArray);

      SensorOutputMapReadOnly sensorOutputMapReadOnly = initializeSensorOutputMapReadOnly();
      RobotConfigurationDataPublisherFactory factory = new RobotConfigurationDataPublisherFactory();
      factory.setDefinitionsToPublish(humanoidFullRobotModel);
      factory.setSensorSource(humanoidFullRobotModel, new ForceSensorDataHolder(forceSensorDefinitionList), sensorOutputMapReadOnly);
      factory.setROS2Info(realtimeROS2Node, ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()));
      RobotConfigurationDataPublisher robotConfigurationDataPublisher = factory.createRobotConfigurationDataPublisher();

      PeriodicNonRealtimeThreadScheduler scheduler2 = new PeriodicNonRealtimeThreadScheduler(threadName);
      scheduler2.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            robotConfigurationDataPublisher.write();
         }
      }, 1, TimeUnit.MILLISECONDS);

      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(drcRobotModel, PubSubImplementation.INTRAPROCESS);
      networkProcessor.setupKinematicsToolboxModule(true);
      networkProcessor.start();
   }

   private long timestamp = 0L;

   private SensorOutputMapReadOnly initializeSensorOutputMapReadOnly()
   {
      return new SensorOutputMapReadOnly()
      {
         @Override
         public long getWallTime()
         {
            timestamp += Conversions.millisecondsToNanoseconds(1L);
            return timestamp;
         }

         @Override
         public long getMonotonicTime()
         {
            return timestamp;
         }

         @Override
         public long getSyncTimestamp()
         {
            return timestamp;
         }

         @Override
         public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
         {
            return null;
         }

         @Override
         public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
         {
            return null;
         }

         @Override
         public List<? extends IMUSensorReadOnly> getIMUOutputs()
         {
            return null;
         }

         @Override
         public ForceSensorDataHolderReadOnly getForceSensorOutputs()
         {
            return null;
         }
      };
   }

}
