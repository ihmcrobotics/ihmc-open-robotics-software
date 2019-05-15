package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.net.URI;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class StereoVisionPointCloudPublisher
{
   private static final boolean Debug = false;

   private static final int MAX_NUMBER_OF_POINTS = 200000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));

   private final AtomicReference<PointCloud2> rosPointCloud2ToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullHumanoidRobotModel fullRobotModel;
   private ReferenceFrame stereoVisionPointsFrame = worldFrame;
   private StereoVisionWorldTransformCalculator stereoVisionTransformer = null;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private PPSTimestampOffsetProvider ppsTimestampOffsetProvider = null;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> pointcloudPublisher;

   public StereoVisionPointCloudPublisher(FullHumanoidRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName)
   {
      robotName = modelFactory.getRobotDescription().getName();
      fullRobotModel = modelFactory.createFullRobotModel();

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      pointcloudPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
   }

   public void start()
   {
      executorService.scheduleAtFixedRate(createPublisherTask(), 0L, 1L, TimeUnit.MILLISECONDS);
   }

   public void shutdown()
   {
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
            rosPointCloud2ToPublish.set(pointCloud);

            if (Debug)
               System.out.println("Receiving point cloud, n points: " + pointCloud.getHeight() * pointCloud.getWidth());
         }
      };
   }

   private Runnable createPublisherTask()
   {
      return new Runnable()
      {
         private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
         private final Pose3D sensorPose = new Pose3D();

         @Override
         public void run()
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
            PointCloud2 pointCloud2 = rosPointCloud2ToPublish.getAndSet(null);

            if (pointCloud2 == null)
               return;

            ColorPointCloudData pointCloudData = new ColorPointCloudData(pointCloud2, MAX_NUMBER_OF_POINTS);

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
            pointcloudPublisher.publish(message);
         }
      };
   }

   public static interface StereoVisionWorldTransformCalculator
   {
      public void computeTransformToWorld(FullHumanoidRobotModel fullRobotModel, ReferenceFrame scanPointsFrame, RigidBodyTransform transformToWorldToPack,
                                          Pose3DBasics sensorPoseToPack);
   }
}
