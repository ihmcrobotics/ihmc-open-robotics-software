package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.awt.Color;
import java.net.URI;
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
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class StereoVisionPointCloudPublisher
{
   private static final int MAX_NUMBER_OF_POINTS = 200000;

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));

   private final AtomicReference<ColorPointCloudData> pointCloudDataToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullHumanoidRobotModel fullRobotModel;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private PPSTimestampOffsetProvider ppsTimestampOffsetProvider = null;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> pointcloudPublisher;

   private final ReferenceFrame dataFrame;

   public StereoVisionPointCloudPublisher(FullHumanoidRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName,
                                          RigidBodyTransform transform)
   {
      robotName = modelFactory.getRobotDescription().getName();
      fullRobotModel = modelFactory.createFullRobotModel();

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      pointcloudPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      dataFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("stereoFrame", fullRobotModel.getHead().getBodyFixedFrame(), transform);
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

   private RosPointCloudSubscriber createROSPointCloud2Subscriber()
   {
      return new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
            Point3D[] scanPoints = pointCloudData.getPoints();
            Color[] colors = pointCloudData.getPointColors();
            long timestamp = pointCloud.getHeader().getStamp().totalNsecs();

            System.out.println("Receiving point cloud, n points: " + scanPoints.length);

            pointCloudDataToPublish.set(new ColorPointCloudData(timestamp, scanPoints, colors));
         }
      };
   }

   private Runnable createPublisherTask()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            ColorPointCloudData pointCloudData = pointCloudDataToPublish.getAndSet(null);
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

            RigidBodyTransform transformToWorldFrame = dataFrame.getTransformToWorldFrame();
            StereoVisionPointCloudMessage message = pointCloudData.createStereoVisionPointCloudMessage(transformToWorldFrame, MAX_NUMBER_OF_POINTS);
            System.out.println("Publishing stereo data, number of points: " + (message.getPointCloud().size() / 3));
            pointcloudPublisher.publish(message);
         }
      };
   }

   private class ColorPointCloudData
   {
      private final long timestamp;
      private final Point3D[] pointCloud;
      private final int[] colors;

      public ColorPointCloudData(long timestamp, Point3D[] pointCloud, Color[] colors)
      {
         this.timestamp = timestamp;
         this.pointCloud = pointCloud;
         this.colors = new int[colors.length];

         for (int i = 0; i < colors.length; i++)
         {
            this.getColors()[i] = colors[i].getRGB();
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

      public StereoVisionPointCloudMessage createStereoVisionPointCloudMessage(RigidBodyTransform transform, int maximumSize)
      {
         int numberOfPoints = pointCloud.length;

         Random random = new Random();
         while (numberOfPoints > maximumSize)
         {
            int indexToRemove = random.nextInt(numberOfPoints);
            int lastIndex = numberOfPoints - 1;

            pointCloud[indexToRemove] = pointCloud[lastIndex];
            colors[indexToRemove] = colors[lastIndex];

            numberOfPoints--;
         }

         for (int i = 0; i < numberOfPoints; i++)
            transform.transform(pointCloud[i]);

         long timestamp = this.timestamp;
         float[] pointCloudBuffer = new float[3 * numberOfPoints];
         int[] colorsInteger = new int[numberOfPoints];

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
            pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
            pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

            colorsInteger[i] = colors[i];
         }

         return MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);
      }
   }
}
