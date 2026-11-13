package us.ihmc.sensorProcessing.communication.producers;

import controller_msgs.IMUPacket;
import controller_msgs.RobotConfigurationData;
import controller_msgs.SpatialVectorMessage;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.FloatingJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class RobotConfigurationDataPublisher implements RawOutputWriter
{
   private final FloatingJointStateReadOnly rootJointSensorData;
   private final List<? extends OneDoFJointStateReadOnly> jointSensorData;
   private final List<? extends IMUSensorReadOnly> imuSensorData;
   private final List<? extends ForceSensorDataReadOnly> forceSensorData;

   private final List<RobotFrameDataPublisher> robotFrameDataPublishers = new ArrayList<>();
   private final SensorTimestampHolder timestampHolder;
   private final RobotMotionStatusHolder robotMotionStatusHolder;

   private final RobotConfigurationData robotConfigurationData = new RobotConfigurationData();
   private final ROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;

   private final long publishPeriod;
   private long lastPublishTime = -1;

   // Counter to keep track of configuration data sequence number
   private long sequenceId = 0;

   /**
    * Intended to be instantiated only using {@link RobotConfigurationDataPublisherFactory}.
    *
    * @param ros2Node                the ROS 2 node to create the publisher with.
    * @param outputTopic             the generator to use to create the name of the topic.
    * @param rootJointSensorData     the data provider for the root joint.
    * @param jointSensorData         the data providers for the 1-DoF joints.
    * @param imuSensorData           the data providers for the IMUs.
    * @param forceSensorData         the data providers for the force sensors.
    * @param timestampHolder         the data provider for the timestamps.
    * @param robotMotionStatusHolder the data provider for the robot motion status.
    * @param publishPeriod           period in nanoseconds to publish.
    */
   public RobotConfigurationDataPublisher(ROS2Node ros2Node,
                                          ROS2Topic<?> outputTopic,
                                          FloatingJointStateReadOnly rootJointSensorData,
                                          List<? extends OneDoFJointStateReadOnly> jointSensorData,
                                          List<? extends IMUSensorReadOnly> imuSensorData,
                                          List<? extends ForceSensorDataReadOnly> forceSensorData,
                                          List<? extends ReferenceFrame> frameData,
                                          SensorTimestampHolder timestampHolder,
                                          RobotMotionStatusHolder robotMotionStatusHolder,
                                          long publishPeriod)
   {
      this.rootJointSensorData = rootJointSensorData;
      this.jointSensorData = jointSensorData;
      this.imuSensorData = imuSensorData;
      this.forceSensorData = forceSensorData;
      this.timestampHolder = timestampHolder;
      this.robotMotionStatusHolder = robotMotionStatusHolder;
      this.publishPeriod = publishPeriod;

      robotConfigurationData.setJointNameHash(RobotConfigurationDataFactory.calculateJointNameHash(jointSensorData, forceSensorData, imuSensorData));
      robotConfigurationDataPublisher = ros2Node.createPublisher(StateEstimatorAPI.getRobotConfigurationDataTopic(outputTopic));

      // Create RobotFrameDataPublishers here.
      for (ReferenceFrame frame : frameData)
      {
         robotFrameDataPublishers.add(new RobotFrameDataPublisher(frame, ros2Node, outputTopic));
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void write()
   {
      if (lastPublishTime > 0 && timestampHolder.getMonotonicTime() - lastPublishTime < publishPeriod)
         return;

      lastPublishTime = timestampHolder.getMonotonicTime();

      // Write timestamps
      robotConfigurationData.setWallTime(timestampHolder.getWallTime());
      robotConfigurationData.setMonotonicTime(timestampHolder.getMonotonicTime());
      robotConfigurationData.setSyncTimestamp(timestampHolder.getSyncTimestamp());

      // Write root joint data
      robotConfigurationData.getRootOrientation().getQuaternion().set(rootJointSensorData.getPose().getOrientation());
      robotConfigurationData.getRootPosition().getPoint().set(rootJointSensorData.getPose().getPosition());
      robotConfigurationData.getPelvisAngularVelocity().getVector().set(rootJointSensorData.getTwist().getAngularPart());
      robotConfigurationData.getPelvisLinearVelocity().getVector().set(rootJointSensorData.getTwist().getLinearPart());
      robotConfigurationData.getPelvisLinearAcceleration().getVector().set(rootJointSensorData.getAcceleration().getLinearPart());

      // Write 1-DoF joint data
      robotConfigurationData.getJointAngles().clear();
      robotConfigurationData.getJointVelocities().clear();
      robotConfigurationData.getJointTorques().clear();

      for (int i = 0; i < jointSensorData.size(); i++)
      {
         OneDoFJointStateReadOnly jointSensorOutput = jointSensorData.get(i);
         robotConfigurationData.getJointAngles().add((float) jointSensorOutput.getPosition());
         robotConfigurationData.getJointVelocities().add((float) jointSensorOutput.getVelocity());
         robotConfigurationData.getJointTorques().add((float) jointSensorOutput.getEffort());
      }

      // Write IMU sensor data
      if (imuSensorData != null)
      {
         robotConfigurationData.getImuSensorData().clear();

         for (int i = 0; i < imuSensorData.size(); i++)
         {
            IMUSensorReadOnly imuSensor = imuSensorData.get(i);
            IMUPacket imuPacketToPack = robotConfigurationData.getImuSensorData().add();

            imuPacketToPack.getOrientation().getQuaternion().set(imuSensor.getOrientationMeasurement());
            imuPacketToPack.getAngularVelocity().getVector().set(imuSensor.getAngularVelocityMeasurement());
            imuPacketToPack.getLinearAcceleration().getVector().set(imuSensor.getLinearAccelerationMeasurement());
         }
      }

      // Write force sensor data
      if (forceSensorData != null)
      {
         robotConfigurationData.getForceSensorData().clear();

         for (int i = 0; i < forceSensorData.size(); i++)
         {
            SpatialVectorMessage forceDataToPack = robotConfigurationData.getForceSensorData().add();
            forceDataToPack.getAngularPart().getVector().set(forceSensorData.get(i).getWrench().getAngularPart());
            forceDataToPack.getLinearPart().getVector().set(forceSensorData.get(i).getWrench().getLinearPart());
         }
      }

      // Write robot motion status
      if (robotMotionStatusHolder != null)
         robotConfigurationData.setRobotMotionStatus(robotMotionStatusHolder.getCurrentRobotMotionStatus().toByte());

      // Write last packet info, fill up with -1 since this information is gone since the switch to RTPS
      robotConfigurationData.setLastReceivedPacketTypeId(-1);
      robotConfigurationData.setLastReceivedPacketUniqueId(-1);
      robotConfigurationData.setLastReceivedPacketRobotTimestamp(-1);

      // update sequence id for each new message
      this.sequenceId++;

      // Set the sequence id for the robot configuration data and propagate it to the sensor data
      MessageTools.setRobotConfigurationDataSequenceId(robotConfigurationData, this.sequenceId);

      robotConfigurationDataPublisher.publish(robotConfigurationData);

      // publish robot frame data
      for (int i = 0; i < robotFrameDataPublishers.size(); i++)
      {
         robotFrameDataPublishers.get(i).publish();
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return null;
   }
}
