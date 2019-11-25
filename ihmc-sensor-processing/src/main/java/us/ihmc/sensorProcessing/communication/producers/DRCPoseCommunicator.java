package us.ihmc.sensorProcessing.communication.producers;

import java.util.List;

import controller_msgs.msg.dds.IMUPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate
// consumer
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorTimestampHolder sensorTimestampHolder;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final RobotMotionStatusHolder robotMotionStatusFromController;

   private final RobotConfigurationData robotConfigurationData;

   private final IHMCRealtimeROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;

   public DRCPoseCommunicator(FullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer,
                              MessageTopicNameGenerator publisherTopicNameGenerator, RealtimeRos2Node realtimeRos2Node,
                              SensorTimestampHolder sensorTimestampHolder, SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly,
                              RobotMotionStatusHolder robotMotionStatusFromController, HumanoidRobotSensorInformation sensorInformation)
   {
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorTimestampHolder = sensorTimestampHolder;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      this.robotMotionStatusFromController = robotMotionStatusFromController;

      IMUDefinition[] imuDefinitions = estimatorModel.getIMUDefinitions();

      ForceSensorDefinition[] forceSensorDefinitions = jointConfigurationGathererAndProducer.getForceSensorDefinitions();
      OneDoFJointBasics[] joints = jointConfigurationGathererAndProducer.getJoints();
      robotConfigurationData = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuDefinitions);
      robotConfigurationDataPublisher = ROS2Tools.createPublisher(realtimeRos2Node, RobotConfigurationData.class, publisherTopicNameGenerator);
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
      long wallTime = sensorTimestampHolder.getWallTime();
      long monotonicTime = sensorTimestampHolder.getMonotonicTime();
      long syncTimestamp = sensorTimestampHolder.getSyncTimestamp();
      jointConfigurationGathererAndProducer.packEstimatorJoints(wallTime, monotonicTime, syncTimestamp, robotConfigurationData);

      if (sensorRawOutputMapReadOnly != null)
      {
         robotConfigurationData.getImuSensorData().clear();

         List<? extends IMUSensorReadOnly> imuRawOutputs = sensorRawOutputMapReadOnly.getIMURawOutputs();
         for (int sensorNumber = 0; sensorNumber < imuRawOutputs.size(); sensorNumber++)
         {
            IMUSensorReadOnly imuSensor = imuRawOutputs.get(sensorNumber);
            IMUPacket imuPacketToPack = robotConfigurationData.getImuSensorData().add();

            imuPacketToPack.getOrientation().set(imuSensor.getOrientationMeasurement());
            imuPacketToPack.getAngularVelocity().set(imuSensor.getAngularVelocityMeasurement());
            imuPacketToPack.getLinearAcceleration().set(imuSensor.getLinearAccelerationMeasurement());
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
   }
}