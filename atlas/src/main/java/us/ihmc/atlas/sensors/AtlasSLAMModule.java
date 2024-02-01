package us.ihmc.atlas.sensors;

import java.io.File;
import java.text.DecimalFormat;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

public class AtlasSLAMModule extends SLAMModule
{
   private static final double PELVIS_VELOCITY_STATIONARY_THRESHOLD = 0.001;
   private static final double TOLERANCE_PELVIS_VELOCITY = 0.01;

   private final LinkedList<Boolean> stationaryFlagQueue = new LinkedList<Boolean>();
   private final LinkedList<Boolean> reasonableVelocityFlagQueue = new LinkedList<Boolean>();

   private final AtomicBoolean robotStatus = new AtomicBoolean(false);
   private final AtomicBoolean velocityStatus = new AtomicBoolean(true);

   /**
    * to update corrected sensor frame for robot state estimation.
    */
   protected final AtomicLong latestRobotTimeStamp = new AtomicLong();
   protected IHMCROS2Publisher<StampedPosePacket> estimatedPelvisPublisher = null;
   protected RigidBodyTransform sensorPoseToPelvisTransformer = null;

   public AtlasSLAMModule(ROS2Node ros2Node, Messager messager, DRCRobotModel drcRobotModel, File configurationFile)
   {
      super(ros2Node, messager, AtlasSensorInformation.transformPelvisToD435DepthCamera, configurationFile);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()),
                                                    this::handleRobotConfigurationData);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    FootstepStatusMessage.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()),
                                                    this::handleFootstepStatusMessage);

      estimatedPelvisPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                    StampedPosePacket.class,
                                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()));
      sensorPoseToPelvisTransformer = new RigidBodyTransform(AtlasSensorInformation.transformPelvisToD435DepthCamera);
      sensorPoseToPelvisTransformer.invert();

      reaMessager.addTopicListener(SLAMModuleAPI.SensorStatus, robotStatus::set);
      reaMessager.addTopicListener(SLAMModuleAPI.VelocityLimitStatus, velocityStatus::set);
   }

   public AtlasSLAMModule(Messager messager, DRCRobotModel drcRobotModel)
   {
      super(messager);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()), this::handleRobotConfigurationData);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepStatusMessage.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()), this::handleFootstepStatusMessage);

      estimatedPelvisPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, StampedPosePacket.class,
                                                                    ROS2Tools.getControllerInputTopic(drcRobotModel.getSimpleRobotName()));
      sensorPoseToPelvisTransformer = new RigidBodyTransform(AtlasSensorInformation.transformPelvisToD435DepthCamera);
      sensorPoseToPelvisTransformer.invert();

      reaMessager.addTopicListener(SLAMModuleAPI.SensorStatus, robotStatus::set);
      reaMessager.addTopicListener(SLAMModuleAPI.VelocityLimitStatus, velocityStatus::set);
   }

   @Override
   public void sendCurrentState()
   {
      super.sendCurrentState();

      if (robotStatus != null)
         reaMessager.submitMessage(SLAMModuleAPI.SensorStatus, robotStatus.get());
      if (velocityStatus != null)
         reaMessager.submitMessage(SLAMModuleAPI.VelocityLimitStatus, velocityStatus.get());
   }

   @Override
   protected boolean addFrame(StereoVisionPointCloudMessage pointCloudToCompute)
   {
      boolean stationaryFlag = stationaryFlagQueue.getFirst();
      boolean reasonableVelocityFlag = reasonableVelocityFlagQueue.getFirst();

      if (reasonableVelocityFlag)
      {
         if (stationaryFlag)
         {
            slam.addKeyFrame(pointCloudToCompute, slamParameters.get().getInsertMissInOcTree());
            return true;
         }
         else
         {
            return slam.addFrame(pointCloudToCompute, slamParameters.get().getInsertMissInOcTree());
         }
      }
      else
      {
         return false;
      }
   }

   @Override
   public void queue(StereoVisionPointCloudMessage pointCloud)
   {
      super.queue(pointCloud);
      stationaryFlagQueue.add(robotStatus.get());
      reasonableVelocityFlagQueue.add(velocityStatus.get());
   }

   @Override
   protected void dequeue()
   {
      super.dequeue();
      if (!stationaryFlagQueue.isEmpty())
         stationaryFlagQueue.removeFirst();
      if (!reasonableVelocityFlagQueue.isEmpty())
         reasonableVelocityFlagQueue.removeFirst();
   }

   @Override
   protected void publishResults()
   {
      super.publishResults();

      if (estimatedPelvisPublisher != null)
      {
         SLAMFrame latestFrame = slam.getLatestFrame();
         if (latestFrame == null)
            return;

         StampedPosePacket posePacket = new StampedPosePacket();
         posePacket.setTimestamp(latestRobotTimeStamp.get());
         int maximumBufferOfQueue = 10;
         if (pointCloudQueue.size() >= maximumBufferOfQueue)
         {
            posePacket.setConfidenceFactor(0.0);
         }
         else
         {
            if (latestFrame.getConfidenceFactor() < 0)
               posePacket.setConfidenceFactor(0.0);
            posePacket.setConfidenceFactor(latestFrame.getConfidenceFactor());
         }
         posePacket.setConfidenceFactor(0.5);
         RigidBodyTransform estimatedPelvisPose = new RigidBodyTransform(sensorPoseToPelvisTransformer);
         estimatedPelvisPose.preMultiply(latestFrame.getCorrectedSensorPoseInWorld());
         posePacket.getPose().set(estimatedPelvisPose);
         reaMessager.submitMessage(SLAMModuleAPI.CustomizedFrameState, posePacket);

         LogTools.debug("latestFrame.getConfidenceFactor: " + latestFrame.getConfidenceFactor() +
                       " posePacket.getConfidenceFactor: " + posePacket.getConfidenceFactor());
         LogTools.debug("publishing pose to state estimator: " + posePacket.getPose());
         estimatedPelvisPublisher.publish(posePacket);
      }
   }

   @Override
   public void clearSLAM()
   {
      super.clearSLAM();
      stationaryFlagQueue.clear();
      reasonableVelocityFlagQueue.clear();
   }

   private final DecimalFormat df = new DecimalFormat("#.####");

   private void handleRobotConfigurationData(Subscriber<RobotConfigurationData> subscriber)
   {
      RobotConfigurationData robotConfigurationData = subscriber.takeNextData();
      latestRobotTimeStamp.set(robotConfigurationData.getWallTime());

      if (reaMessager.isMessagerOpen())
      {
         double pelvisVelocity = robotConfigurationData.getPelvisLinearVelocity().lengthSquared();
         reaMessager.submitMessage(SLAMModuleAPI.SensorStatus, pelvisVelocity < slamParameters.get().getStationaryVelocity());
         reaMessager.submitMessage(SLAMModuleAPI.VelocityLimitStatus, pelvisVelocity < slamParameters.get().getMaxVelocity());
         reaMessager.submitMessage(SLAMModuleAPI.SensorSpeed, "" + df.format(Math.sqrt(pelvisVelocity)));
      }
   }

   private void handleFootstepStatusMessage(Subscriber<FootstepStatusMessage> subscriber)
   {
      FootstepStatusMessage footstepStatusMessage = subscriber.takeNextData();
      if (reaMessager.isMessagerOpen())
      {
         if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            reaMessager.submitMessage(SLAMModuleAPI.ShowFootstepDataViz, true);
            RobotSide robotSide = RobotSide.fromByte(footstepStatusMessage.getRobotSide());
            Point3DReadOnly footLocation = footstepStatusMessage.getActualFootPositionInWorld();
            QuaternionReadOnly footOrientation = footstepStatusMessage.getActualFootOrientationInWorld();

            FootstepDataMessage footstepDataMessageToSubmit = new FootstepDataMessage();
            footstepDataMessageToSubmit.setRobotSide(robotSide.toByte());
            footstepDataMessageToSubmit.getLocation().set(footLocation);
            footstepDataMessageToSubmit.getOrientation().set(footOrientation);
            reaMessager.submitMessage(SLAMModuleAPI.FootstepDataState, footstepDataMessageToSubmit);
         }
      }
   }

   public static AtlasSLAMModule createIntraprocessModule(DRCRobotModel drcRobotModel, Messager messager)
   {
      return new AtlasSLAMModule(messager, drcRobotModel);
   }

   public static AtlasSLAMModule createIntraprocessModule(ROS2Node ros2Node, DRCRobotModel drcRobotModel, Messager messager, File configurationFile)
   {
      return new AtlasSLAMModule(ros2Node, messager, drcRobotModel, configurationFile);
   }
}
