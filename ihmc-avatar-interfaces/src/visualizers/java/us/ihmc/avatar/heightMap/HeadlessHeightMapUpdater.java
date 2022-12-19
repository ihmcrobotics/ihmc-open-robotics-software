package us.ihmc.avatar.heightMap;

import org.apache.commons.lang3.tuple.Pair;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeadlessHeightMapUpdater
{
   private final ROS2Node ros2Node;

   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean(false);
   private final AbstractHeightMapUpdater heightMapUpdater;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()),
                                                                                                    ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public HeadlessHeightMapUpdater()
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "height_map");

      IHMCROS2Publisher<HeightMapMessage> heightMapPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);

      heightMapUpdater = new AbstractHeightMapUpdater(heightMapPublisher::publish);

      ROS2Tools.createCallbackSubscription(ros2Node, LidarScanMessage.class, ROS2Tools.OUSTER_LIDAR_SCAN, new NewMessageListener<LidarScanMessage>()
      {
         @Override
         public void onNewDataMessage(Subscriber<LidarScanMessage> subscriber)
         {
            LidarScanMessage data = subscriber.readNextData();
            //            FramePose3D ousterPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());

            PointCloudData pointCloudData = new PointCloudData(data);
            heightMapUpdater.addPointCloudToQueue(Pair.of(pointCloudData, new FramePose3D()));
         }
      });


      int initialPublishFrequency = 5;
      heightMapUpdater.setPublishFrequency(initialPublishFrequency);
      heightMapUpdater.setEnableUpdates(true);
      heightMapUpdater.setGridCenterX(2.0);
      heightMapUpdater.setGridCenterY(0.0);

      executorService.scheduleAtFixedRate(this::update, 0, 100, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      if (heightMapUpdater.updatesAreEnabled())
      {
         if (!updateThreadIsRunning.getAndSet(true))
         {
            heightMapUpdater.runUpdateThread();
            updateThreadIsRunning.set(false);
         }
      }
   }

   public void stop()
   {
      ros2Node.destroy();
      executorService.shutdown();
   }

   public static void main(String[] args)
   {
      new HeadlessHeightMapUpdater();
   }
}
