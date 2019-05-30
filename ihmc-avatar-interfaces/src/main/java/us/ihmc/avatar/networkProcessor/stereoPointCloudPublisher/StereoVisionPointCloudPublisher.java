package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

import java.net.URI;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class StereoVisionPointCloudPublisher
{
   private static final boolean Debug = false;

   private static final int MAX_NUMBER_OF_POINTS = 200000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<ColorPointCloudData> rosPointCloud2ToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullRobotModel fullRobotModel;
   private ReferenceFrame stereoVisionPointsFrame = worldFrame;
   private StereoVisionWorldTransformCalculator stereoVisionTransformer = null;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private PPSTimestampOffsetProvider ppsTimestampOffsetProvider = null;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> pointcloudPublisher;
   private final IHMCRealtimeROS2Publisher<StereoVisionPointCloudMessage> pointcloudRealtimePublisher;

   public StereoVisionPointCloudPublisher(FullRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node, null, robotConfigurationDataTopicName);
   }


   public StereoVisionPointCloudPublisher(String robotName, FullRobotModel fullRobotModel, RealtimeRos2Node ros2Node, String robotConfigurationDataTopicName)
   {
      this(robotName, fullRobotModel, null, ros2Node, robotConfigurationDataTopicName);
   }

   public StereoVisionPointCloudPublisher(String robotName, FullRobotModel fullRobotModel, Ros2Node ros2Node, RealtimeRos2Node realtimeRos2Node,
                                          String robotConfigurationDataTopicName)
   {
      this.robotName = robotName;
      this.fullRobotModel = fullRobotModel;

      if (ros2Node != null)
      {
         ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         pointcloudPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
         pointcloudRealtimePublisher = null;
      }
      else
      {
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         pointcloudPublisher = null;
         pointcloudRealtimePublisher = ROS2Tools.createPublisher(realtimeRos2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

      }
   }

   public void start()
   {
      publisherTask = executorService.scheduleAtFixedRate(this::readAndPublishInternal, 0L, 1L, TimeUnit.MILLISECONDS);
   }

   public void shutdown()
   {
      publisherTask.cancel(false);
      executorService.shutdownNow();
   }

   public void receiveStereoPointCloudFromROS(String stereoPointCloudROSTopic, URI rosCoreURI)
   {
      String graphName = robotName + "/" + name;
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, graphName, true);
      receiveStereoPointCloudFromROS(stereoPointCloudROSTopic, rosMainNode);
   }

   public void receiveStereoPointCloudFromROS(String stereoPointCloudROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(stereoPointCloudROSTopic, createROSPointCloud2Subscriber());
   }

   public void setPPSTimestampOffsetProvider(PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
   }

   public void setCustomStereoVisionTransformer(StereoVisionWorldTransformCalculator transformer)
   {
      LogTools.info("setCustomStereoVisionTransformer()");
      stereoVisionTransformer = transformer;
   }

   private RosPointCloudSubscriber createROSPointCloud2Subscriber()
   {
      return new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            rosPointCloud2ToPublish.set(new ColorPointCloudData(pointCloud, MAX_NUMBER_OF_POINTS));

            if (Debug)
               System.out.println("Receiving point cloud, n points: " + pointCloud.getHeight() * pointCloud.getWidth());
         }
      };
   }

   public void updateScanData(ColorPointCloudData scanDataToPublish)
   {
      this.rosPointCloud2ToPublish.set(scanDataToPublish);
   }

   public void readAndPublish()
   {
      if (publisherTask != null)
         throw new RuntimeException("The publisher is running using its own thread, cannot manually update it.");

      readAndPublishInternal();
   }

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final Pose3D sensorPose = new Pose3D();

   private void readAndPublishInternal()
   {
      try
      {
         transformDataAndPublish();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         executorService.shutdown();
      }
   }

   private void transformDataAndPublish()
   {
      ColorPointCloudData pointCloudData = rosPointCloud2ToPublish.getAndSet(null);

      if (pointCloudData == null)
         return;

      long robotTimestamp;

      if (ppsTimestampOffsetProvider == null)
      {
         robotTimestamp = pointCloudData.getTimestamp();
         robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
      }
      else
      {
         long timestamp = pointCloudData.getTimestamp();
         robotTimestamp = ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timestamp);
         boolean waitForTimestamp = true;
         boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;
         if (!success)
            return;
      }

      if (stereoVisionTransformer != null)
      {
         stereoVisionTransformer.computeTransformToWorld(fullRobotModel, stereoVisionPointsFrame, transformToWorld, sensorPose);
         pointCloudData.applyTransform(transformToWorld);
      }
      else
      {
         if (!stereoVisionPointsFrame.isWorldFrame())
         {
            stereoVisionPointsFrame.getTransformToDesiredFrame(transformToWorld, worldFrame);
            pointCloudData.applyTransform(transformToWorld);
         }

         fullRobotModel.getHeadBaseFrame().getTransformToDesiredFrame(transformToWorld, worldFrame);
         sensorPose.set(transformToWorld);
      }

      StereoVisionPointCloudMessage message = pointCloudData.toStereoVisionPointCloudMessage();
      message.getSensorPosition().set(sensorPose.getPosition());
      message.getSensorOrientation().set(sensorPose.getOrientation());

      if (Debug)
         System.out.println("Publishing stereo data, number of points: " + (message.getPointCloud().size() / 3));
      if (pointcloudPublisher != null)
         pointcloudPublisher.publish(message);
      else
         pointcloudRealtimePublisher.publish(message);
   }

   public static interface StereoVisionWorldTransformCalculator
   {
      public void computeTransformToWorld(FullRobotModel fullRobotModel, ReferenceFrame scanPointsFrame, RigidBodyTransform transformToWorldToPack,
                                          Pose3DBasics sensorPoseToPack);
   }
}
