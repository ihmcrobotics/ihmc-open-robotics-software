package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.awt.Color;
import java.net.URI;
import java.util.EnumSet;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.list.array.TFloatArrayList;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.RequestStereoPointCloudMessage;
import us.ihmc.communication.packets.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class StereoVisionPointCloudPublisher
{
   private static final int MAX_NUMBER_OF_LISTENERS = 10;

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));

   private final AtomicReference<ColorPointCloudData> pointCloudDataToPublish = new AtomicReference<>(null);

   private final PacketCommunicator packetCommunicator;
   private final ConcurrentLinkedDeque<PacketDestination> listeners = new ConcurrentLinkedDeque<>();

   private final String robotName;
   private final FullHumanoidRobotModel fullRobotModel;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private PPSTimestampOffsetProvider ppsTimestampOffsetProvider = null;

   public StereoVisionPointCloudPublisher(FullHumanoidRobotModelFactory modelFactory, PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      robotName = modelFactory.getRobotDescription().getName();
      fullRobotModel = modelFactory.createFullRobotModel();

      packetCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      packetCommunicator.attachListener(RequestStereoPointCloudMessage.class, createRequestStereoPointCloudMessageConsumer());
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

            if (listeners.isEmpty())
               return;

            int count = 0;
            Set<PacketDestination> listenerSet = EnumSet.noneOf(PacketDestination.class);

            while (!listeners.isEmpty() && count < MAX_NUMBER_OF_LISTENERS)
            {
               listenerSet.add(listeners.poll());
               count++;
            }
            listeners.clear();

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

            for (PacketDestination destination : listenerSet)
            {
               float[] scanPointBuffer = pointCloudData.getPointCloudBuffer();

               int[] colors = pointCloudData.getColors();
               StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage(robotTimestamp, scanPointBuffer, colors);

               message.setDestination(destination);
               packetCommunicator.send(message);
            }

            listeners.clear();
         }
      };

   }

   private PacketConsumer<RequestStereoPointCloudMessage> createRequestStereoPointCloudMessageConsumer()
   {
      return new PacketConsumer<RequestStereoPointCloudMessage>()
      {
         @Override
         public void receivedPacket(RequestStereoPointCloudMessage packet)
         {
            if (packet != null)
               listeners.add(PacketDestination.fromOrdinal(packet.getSource()));
         }
      };
   }

   private class ColorPointCloudData
   {
      private final long timestamp;
      private final Point3D[] pointCloud;
      private final int numberOfPoints;
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

         numberOfPoints = pointCloud.length;
      }

      public long getTimestamp()
      {
         return timestamp;
      }

      public void transform(RigidBodyTransform transform)
      {
         for (int i = 0; i < numberOfPoints; i++)
            transform.transform(pointCloud[i]);
      }

      public float[] getPointCloudBuffer()
      {
         TFloatArrayList pointCloudBuffer = new TFloatArrayList();

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            pointCloudBuffer.add((float) scanPoint.getX());
            pointCloudBuffer.add((float) scanPoint.getY());
            pointCloudBuffer.add((float) scanPoint.getZ());
         }

         return pointCloudBuffer.toArray();
      }

      public int[] getColors()
      {
         return colors;
      }

   }
}
