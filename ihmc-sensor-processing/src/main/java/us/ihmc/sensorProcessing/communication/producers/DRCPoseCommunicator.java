package us.ihmc.sensorProcessing.communication.producers;

import java.util.List;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
import controller_msgs.msg.dds.IMUPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.simulatedSensors.AuxiliaryRobotDataProvider;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate
// consumer
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorTimestampHolder sensorTimestampHolder;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   //   private final SideDependentList<String> wristForceSensorNames;
   private final RobotMotionStatusHolder robotMotionStatusFromController;

   // puts the state data into the ring buffer for the output thread
   private final Vector3D32[] imuLinearAccelerations;
   private final Vector3D32[] rawImuAngularVelocities;
   private final Quaternion32[] imuOrientations;

   private final SideDependentList<ReferenceFrame> wristForceSensorFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ForceSensorDistalMassCompensator> wristForceSensorDistalMassCompensators = new SideDependentList<ForceSensorDistalMassCompensator>();

   private final RobotConfigurationData robotConfigurationData;
   private final AtlasAuxiliaryRobotData atlasAuxiliaryRobotData = new AtlasAuxiliaryRobotData();

   private final IHMCRealtimeROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final IHMCRealtimeROS2Publisher<AtlasAuxiliaryRobotData> atlasAuxiliaryRobotDataPublisher;

   public DRCPoseCommunicator(FullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer,
                              AuxiliaryRobotDataProvider auxiliaryRobotDataProvider, MessageTopicNameGenerator publisherTopicNameGenerator,
                              RealtimeRos2Node realtimeRos2Node, SensorTimestampHolder sensorTimestampHolder,
                              SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly, RobotMotionStatusHolder robotMotionStatusFromController,
                              HumanoidRobotSensorInformation sensorInformation)
   {
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorTimestampHolder = sensorTimestampHolder;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      this.robotMotionStatusFromController = robotMotionStatusFromController;

      if (sensorInformation != null)
      {
         SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
         if (wristForceSensorNames != null && !wristForceSensorNames.isEmpty())
         {
            setupForceSensorMassCompensators(estimatorModel, wristForceSensorNames);
         }
      }

      IMUDefinition[] imuDefinitions = estimatorModel.getIMUDefinitions();

      int numberOfImuSensors = imuDefinitions.length;
      imuLinearAccelerations = new Vector3D32[numberOfImuSensors];
      rawImuAngularVelocities = new Vector3D32[numberOfImuSensors];
      imuOrientations = new Quaternion32[numberOfImuSensors];

      for (int imuSensorIndex = 0; imuSensorIndex < numberOfImuSensors; imuSensorIndex++)
      {
         imuLinearAccelerations[imuSensorIndex] = new Vector3D32();
         rawImuAngularVelocities[imuSensorIndex] = new Vector3D32();
         imuOrientations[imuSensorIndex] = new Quaternion32();
      }

      ForceSensorDefinition[] forceSensorDefinitions = jointConfigurationGathererAndProducer.getForceSensorDefinitions();
      OneDoFJointBasics[] joints = jointConfigurationGathererAndProducer.getJoints();
      robotConfigurationData = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuDefinitions);
      robotConfigurationDataPublisher = ROS2Tools.createPublisher(realtimeRos2Node, RobotConfigurationData.class, publisherTopicNameGenerator);

      if (auxiliaryRobotDataProvider == null)
      {
         atlasAuxiliaryRobotDataPublisher = null;
      }
      else
      {
         atlasAuxiliaryRobotDataPublisher = ROS2Tools.createPublisher(realtimeRos2Node, AtlasAuxiliaryRobotData.class, publisherTopicNameGenerator);
      }
   }

   private void setupForceSensorMassCompensators(FullRobotModel estimatorModel, SideDependentList<String> wristForceSensorNames)
   {
      ForceSensorDefinition[] forceSensorDefinitions = estimatorModel.getForceSensorDefinitions();

      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         ForceSensorDefinition sensorDef = forceSensorDefinitions[i];
         String forceSensorName = sensorDef.getSensorName();

         for (RobotSide robotSide : RobotSide.values)
         {
            if (forceSensorName.equals(wristForceSensorNames.get(robotSide)))
            {
               ForceSensorDistalMassCompensator massComp = new ForceSensorDistalMassCompensator(sensorDef, WORKER_SLEEP_TIME_MILLIS, registry);
               wristForceSensorDistalMassCompensators.put(robotSide, massComp);

               ReferenceFrame sensorFrame = sensorDef.getSensorFrame();
               wristForceSensorFrames.put(robotSide, sensorFrame);
            }
         }
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   // puts the state data into the ring buffer for the output thread
   @Override
   public void write()
   {
      long timestamp = sensorTimestampHolder.getVisionSensorTimestamp();
      long pps = sensorTimestampHolder.getSensorHeadPPSTimestamp();
      jointConfigurationGathererAndProducer.packEstimatorJoints(timestamp, pps, robotConfigurationData);

      if (sensorRawOutputMapReadOnly != null)
      {
         robotConfigurationData.getImuSensorData().clear();

         List<? extends IMUSensorReadOnly> imuRawOutputs = sensorRawOutputMapReadOnly.getIMURawOutputs();
         for (int sensorNumber = 0; sensorNumber < imuRawOutputs.size(); sensorNumber++)
         {
            IMUSensorReadOnly imuSensor = imuRawOutputs.get(sensorNumber);
            IMUPacket imuPacketToPack = robotConfigurationData.getImuSensorData().add();

            imuLinearAccelerations[sensorNumber].set(imuSensor.getLinearAccelerationMeasurement());
            imuOrientations[sensorNumber].set(imuSensor.getOrientationMeasurement());
            rawImuAngularVelocities[sensorNumber].set(imuSensor.getAngularVelocityMeasurement());

            imuPacketToPack.getLinearAcceleration().set(imuLinearAccelerations[sensorNumber]);
            imuPacketToPack.getOrientation().set(imuOrientations[sensorNumber]);
            imuPacketToPack.getAngularVelocity().set(rawImuAngularVelocities[sensorNumber]);
         }
      }

      robotConfigurationData.setRobotMotionStatus(robotMotionStatusFromController.getCurrentRobotMotionStatus().toByte());

      // FIXME lost the knowledge of the last packet received when switching to DDS.
      //         LastPacket lastPacket = null;
      //         if (lastPacket != null)
      //         {
      //            configData.setLastReceivedPacketTypeId(netClassList.getID(lastPacket.getPacket()));
      //            configData.setLastReceivedPacketUniqueId(lastPacket.getUniqueId());
      //            configData.setLastReceivedPacketRobotTimestamp(lastPacket.getReceivedTimestamp());
      //         }
      //         else
      {
         robotConfigurationData.setLastReceivedPacketTypeId(-1);
         robotConfigurationData.setLastReceivedPacketUniqueId(-1);
         robotConfigurationData.setLastReceivedPacketRobotTimestamp(-1);
      }

      robotConfigurationDataPublisher.publish(robotConfigurationData);

      if (atlasAuxiliaryRobotDataPublisher != null && sensorRawOutputMapReadOnly != null)
      {
         AtlasAuxiliaryRobotData newData = sensorRawOutputMapReadOnly.getAuxiliaryRobotData();
         if (newData != null)
         {
            atlasAuxiliaryRobotData.set(newData);
            atlasAuxiliaryRobotDataPublisher.publish(atlasAuxiliaryRobotData);
         }
      }
   }
}