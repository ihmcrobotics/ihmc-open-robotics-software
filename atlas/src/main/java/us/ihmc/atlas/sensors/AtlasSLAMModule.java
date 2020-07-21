package us.ihmc.atlas.sensors;

import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.*;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasSLAMModule extends SLAMModule
{
   private static final double PELVIS_VELOCITY_STATIONARY_THRESHOLD = 0.001;
   private static final double TOLERANCE_PELVIS_VELOCITY = 0.01;

   private final LinkedList<Boolean> stationaryFlagQueue = new LinkedList<Boolean>();
   private final LinkedList<Boolean> reasonableVelocityFlagQueue = new LinkedList<Boolean>();

   private final AtomicReference<Boolean> robotStatus;
   private final AtomicReference<Boolean> velocityStatus;
   
   private final AtomicReference<Boolean> biasEnable;

   /**
    * to update corrected sensor frame for robot state estimation.
    */
   protected final AtomicLong latestRobotTimeStamp = new AtomicLong();
   protected IHMCROS2Publisher<StampedPosePacket> estimatedPelvisPublisher = null;
   protected RigidBodyTransform sensorPoseToPelvisTransformer = null;

   public AtlasSLAMModule(Messager messager, DRCRobotModel drcRobotModel)
   {
      super(messager);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()),
                                                    this::handleRobotConfigurationData);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    FootstepStatusMessage.class,
                                                    //ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()),
                                                    ROS2Tools.getControllerInputTopic(drcRobotModel.getSimpleRobotName()),
                                                    this::handleFootstepStatusMessage);

      estimatedPelvisPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                    StampedPosePacket.class,
                                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()));
      sensorPoseToPelvisTransformer = new RigidBodyTransform(AtlasSensorInformation.transformPelvisToDepthCamera);
      sensorPoseToPelvisTransformer.invert();

      robotStatus = reaMessager.createInput(SLAMModuleAPI.SensorStatus, false);
      velocityStatus = reaMessager.createInput(SLAMModuleAPI.VelocityLimitStatus, true);
      biasEnable = reaMessager.createInput(SLAMModuleAPI.BiasEnable, false);
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
            slam.addKeyFrame(pointCloudToCompute);
            return true;
         }
         else
         {
            return slam.addFrame(pointCloudToCompute);
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
      stationaryFlagQueue.removeFirst();
      reasonableVelocityFlagQueue.removeFirst();
   }

   @Override
   protected void publishResults()
   {
      super.publishResults();

      if (estimatedPelvisPublisher != null)
      {
         SLAMFrame latestFrame = slam.getLatestFrame();
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
         posePacket.setConfidenceFactor(1.0);
         RigidBodyTransform estimatedPelvisPose = new RigidBodyTransform(sensorPoseToPelvisTransformer);
         estimatedPelvisPose.preMultiply(latestFrame.getSensorPose());
         posePacket.getPose().set(estimatedPelvisPose);
         reaMessager.submitMessage(SLAMModuleAPI.CustomizedFrameState, posePacket);

         LogTools.info(""+latestFrame.getConfidenceFactor() + " " + posePacket.getConfidenceFactor());
         if(biasEnable.get())
         {
            posePacket.getPose().getPosition().addZ(0.5);
            estimatedPelvisPublisher.publish(posePacket);
            LogTools.info("publishing biased " + posePacket.getPose().getPosition());
         }
         else
         {
            estimatedPelvisPublisher.publish(posePacket);
            LogTools.info("publishing "+ posePacket.getPose().getPosition());
         }
      }
   }


   @Override
   public void clearSLAM()
   {
      super.clearSLAM();
      stationaryFlagQueue.clear();
      reasonableVelocityFlagQueue.clear();
   }

   private void handleRobotConfigurationData(Subscriber<RobotConfigurationData> subscriber)
   {
      RobotConfigurationData robotConfigurationData = subscriber.takeNextData();
      latestRobotTimeStamp.set(robotConfigurationData.getMonotonicTime());

      if (reaMessager.isMessagerOpen())
      {
         if (robotConfigurationData.getPelvisLinearVelocity().lengthSquared() < PELVIS_VELOCITY_STATIONARY_THRESHOLD)
         {
            reaMessager.submitMessage(SLAMModuleAPI.SensorStatus, true);
         }
         else
         {
            reaMessager.submitMessage(SLAMModuleAPI.SensorStatus, false);
         }

         if (robotConfigurationData.getPelvisLinearVelocity().lengthSquared() < TOLERANCE_PELVIS_VELOCITY)
         {
            reaMessager.submitMessage(SLAMModuleAPI.VelocityLimitStatus, true);
         }
         else
         {
            reaMessager.submitMessage(SLAMModuleAPI.VelocityLimitStatus, false);
         }
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

   public static AtlasSLAMModule createIntraprocessModule(DRCRobotModel drcRobotModel) throws Exception
   {
      KryoMessager messager = KryoMessager.createIntraprocess(SLAMModuleAPI.API,
                                                              NetworkPorts.SLAM_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();

      return new AtlasSLAMModule(messager, drcRobotModel);
   }
}
