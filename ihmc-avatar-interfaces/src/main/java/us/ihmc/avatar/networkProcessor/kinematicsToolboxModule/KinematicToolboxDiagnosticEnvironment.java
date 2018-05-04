package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.AuxiliaryRobotDataProvider;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class KinematicToolboxDiagnosticEnvironment
{
   private final String threadName = "NonRealtimeScheduler";

   public KinematicToolboxDiagnosticEnvironment(DRCRobotModel drcRobotModel)
   {
      FullHumanoidRobotModel humanoidFullRobotModel = drcRobotModel.createFullRobotModel();
      DRCRobotJointMap jointMap = drcRobotModel.getJointMap();
      HumanoidFloatingRootJointRobot humanoidFloatingRobotModel = drcRobotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = drcRobotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      robotInitialSetup.initializeRobot(humanoidFloatingRobotModel, jointMap);
      SDFPerfectSimulatedSensorReader sdfPerfectReader = new SDFPerfectSimulatedSensorReader(humanoidFloatingRobotModel, humanoidFullRobotModel,
                                                                                             null);
      sdfPerfectReader.read();

      ForceSensorDefinition[] forceSensorDefinitionArray = humanoidFullRobotModel.getForceSensorDefinitions();
      List<ForceSensorDefinition> forceSensorDefinitionList = Arrays.asList(forceSensorDefinitionArray);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitionList);
      JointConfigurationGatherer jointConfigurationGatherer = new JointConfigurationGatherer(humanoidFullRobotModel, forceSensorDataHolder);

      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      PacketCommunicator controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, netClassList);
      try
      {
         controllerPacketCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      AuxiliaryRobotDataProvider auxiliaryRobotDataProvider = initializeAuxiliaryRobotDataProvider();
      HumanoidGlobalDataProducer dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicator);
      SensorOutputMapReadOnly sensorOutputMapReadOnly = initializeSensorOutputMapReadOnly();
      SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly = initializeSensorRawOutputMapReadOnly();
      RobotMotionStatusHolder robotMotionStatusFromController = new RobotMotionStatusHolder();
      DRCRobotSensorInformation sensorInformation = drcRobotModel.getSensorInformation();
      PeriodicNonRealtimeThreadScheduler scheduler1 = new PeriodicNonRealtimeThreadScheduler(threadName);
      final DRCPoseCommunicator poseCommunicator = new DRCPoseCommunicator(humanoidFullRobotModel, jointConfigurationGatherer, auxiliaryRobotDataProvider,
                                                                     dataProducer, sensorOutputMapReadOnly, sensorRawOutputMapReadOnly,
                                                                     robotMotionStatusFromController, sensorInformation, scheduler1, netClassList);
      PeriodicNonRealtimeThreadScheduler scheduler2 = new PeriodicNonRealtimeThreadScheduler(threadName);
      scheduler2.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            poseCommunicator.write();
         }
      }, 1, TimeUnit.MILLISECONDS);

      DRCNetworkModuleParameters parameters = new DRCNetworkModuleParameters();
      parameters.enableNetworkProcessor(true);
      parameters.enableUiModule(true);
      parameters.enableKinematicsToolbox(true);
      parameters.enableKinematicsToolboxVisualizer(true);
      parameters.enableLocalControllerCommunicator(true);
      new DRCNetworkProcessor(drcRobotModel, parameters);
   }

   private AuxiliaryRobotDataProvider initializeAuxiliaryRobotDataProvider()
   {
      return new AuxiliaryRobotDataProvider()
      {

         @Override
         public AtlasAuxiliaryRobotData newAuxiliaryRobotDataInstance()
         {
            return null;
         }
      };
   }

   private SensorRawOutputMapReadOnly initializeSensorRawOutputMapReadOnly()
   {
      return new SensorRawOutputMapReadOnly()
      {

         @Override
         public long getVisionSensorTimestamp()
         {
            return 0;
         }

         @Override
         public long getTimestamp()
         {
            return 0;
         }

         @Override
         public long getSensorHeadPPSTimestamp()
         {
            return 0;
         }

         @Override
         public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointTauRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public List<? extends IMUSensorReadOnly> getIMURawOutputs()
         {
            return Collections.<IMUSensorReadOnly>emptyList();
         }

         @Override
         public ForceSensorDataHolderReadOnly getForceSensorRawOutputs()
         {
            return null;
         }

         @Override
         public AtlasAuxiliaryRobotData getAuxiliaryRobotData()
         {
            return null;
         }
      };
   }

   private long timestamp = 0L;

   private SensorOutputMapReadOnly initializeSensorOutputMapReadOnly()
   {
      return new SensorOutputMapReadOnly()
      {

         @Override
         public long getVisionSensorTimestamp()
         {
            timestamp += Conversions.millisecondsToNanoseconds(1L);
            return timestamp;
         }

         @Override
         public long getTimestamp()
         {
            return timestamp;
         }

         @Override
         public long getSensorHeadPPSTimestamp()
         {
            return timestamp;
         }

         @Override
         public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
         {
            return false;
         }

         @Override
         public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
         {
            return null;
         }

         @Override
         public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
         {
            return null;
         }
      };
   }

}
