package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.net.URI;
import java.util.Arrays;
import java.util.Random;
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
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

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
   private StereoVisionTransformer stereoVisionTransformer = null;

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

   public void setCustomStereoVisionTransformer(StereoVisionTransformer transformer)
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

         @Override
         public void run()
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
               stereoVisionTransformer.transform(fullRobotModel, stereoVisionPointsFrame, pointCloudData);
            }
            else if (!stereoVisionPointsFrame.isWorldFrame())
            {
               stereoVisionPointsFrame.getTransformToDesiredFrame(transformToWorld, worldFrame);
               pointCloudData.applyTransform(transformToWorld);
            }

            StereoVisionPointCloudMessage message = pointCloudData.createStereoVisionPointCloudMessage(MAX_NUMBER_OF_POINTS);
            if (Debug)
               System.out.println("Publishing stereo data, number of points: " + (message.getPointCloud().size() / 3));
            pointcloudPublisher.publish(message);
         }
      };
   }

   public class ColorPointCloudData
   {
      private final long timestamp;
      private final int numberOfPoints;
      private final Point3D[] pointCloud;
      private final int[] colors;

      public ColorPointCloudData(PointCloud2 rosPointCloud2, int maxSize)
      {
         timestamp = rosPointCloud2.getHeader().getStamp().totalNsecs();

         UnpackedPointCloud unpackPointsAndIntensities = RosPointCloudSubscriber.unpackPointsAndIntensities(rosPointCloud2);
         pointCloud = unpackPointsAndIntensities.getPoints();
         colors = unpackPointsAndIntensities.getPointColorsRGB();

         if (unpackPointsAndIntensities.getPoints().length <= maxSize)
         {
            numberOfPoints = pointCloud.length;
         }
         else
         {
            Random random = new Random();
            int currentSize = pointCloud.length;

            while (currentSize > maxSize)
            {
               int nextToRemove = random.nextInt(currentSize);
               pointCloud[nextToRemove] = pointCloud[currentSize - 1];
               colors[nextToRemove] = colors[currentSize - 1];
               pointCloud[currentSize - 1] = null;
               colors[currentSize - 1] = -1;

               currentSize--;
            }
            numberOfPoints = maxSize;
         }
      }

      public long getTimestamp()
      {
         return timestamp;
      }

      public int[] getColors()
      {
         return colors;
      }

      public StereoVisionPointCloudMessage createStereoVisionPointCloudMessage(int maximumSize)
      {
         long timestamp = this.timestamp;
         float[] pointCloudBuffer = new float[3 * numberOfPoints];
         int[] colorsInteger;

         if (colors.length == numberOfPoints)
            colorsInteger = colors;
         else
            colorsInteger = Arrays.copyOf(colors, numberOfPoints);

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
            pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
            pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();
         }

         return MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);
      }

      public void applyTransform(RigidBodyTransform transform)
      {
         for (Point3D point : pointCloud)
         {
            point.applyTransform(transform);
         }
      }
   }

   public static interface StereoVisionTransformer
   {
      public void transform(FullHumanoidRobotModel fullRobotModel, ReferenceFrame scanPointsFrame, ColorPointCloudData scanDataToTransformToWorld);
   }
}
