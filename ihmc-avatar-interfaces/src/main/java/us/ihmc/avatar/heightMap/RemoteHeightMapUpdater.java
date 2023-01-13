package us.ihmc.avatar.heightMap;

import org.apache.commons.lang3.tuple.Triple;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class RemoteHeightMapUpdater
{
   private final RealtimeROS2Node ros2Node;

   private static final long updateDTMillis = 100;
   private static final int initialPublishFrequency = 5;

   private static final FramePose3D zeroPose = new FramePose3D();
   private final PerceptionMessageTools perceptionMessageTools = new PerceptionMessageTools();
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean(false);
   private final HeightMapUpdater heightMapUpdater;
   private ROS2SyncedRobotModel syncedRobot;

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()),
                                                                                                    ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   public RemoteHeightMapUpdater(DRCRobotModel robotModel)
   {
      ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "height_map");
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      IHMCRealtimeROS2Publisher<HeightMapMessage > heightMapPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);

      heightMapUpdater = new HeightMapUpdater();
      heightMapUpdater.attachHeightMapConsumer(message ->
            {
               LogTools.info("Publishing height map");
                  heightMapPublisher.publish(message);
            });

      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.OUSTER_LIDAR_SCAN, ROS2QosProfile.BEST_EFFORT(), new NewMessageListener<LidarScanMessage>()
      {
         @Override
         public void onNewDataMessage(Subscriber<LidarScanMessage> subscriber)
         {
            syncedRobot.update();

            double groundHeight = syncedRobot.getReferenceFrames().getMidFeetZUpFrame().getTransformToRoot().getTranslationZ();

            LidarScanMessage data = subscriber.readNextData();
            //            FramePose3D ousterPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());
            Point3D gridCenter = new Point3D(data.getLidarPosition().getX(), data.getLidarPosition().getY(), groundHeight);
            PointCloudData pointCloudData = new PointCloudData(data);
            FramePose3D sensorPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());

            heightMapUpdater.addPointCloudToQueue(Triple.of(pointCloudData, sensorPose, gridCenter));
         }
      });

      /*
      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.OUSTER_DEPTH_IMAGE, ROS2QosProfile.BEST_EFFORT(), new NewMessageListener<ImageMessage>()
      {
         @Override
         public void onNewDataMessage(Subscriber<ImageMessage> subscriber)
         {
            syncedRobot.update();

            double groundHeight = syncedRobot.getReferenceFrames().getMidFeetZUpFrame().getTransformToRoot().getTranslationZ();

            ImageMessage data = subscriber.readNextData();
            //            FramePose3D ousterPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());
            Point3D gridCenter = new Point3D(data.getPosition().getX(), data.getPosition().getY(), groundHeight);
            PointCloudData pointCloudData = new PointCloudData(perceptionMessageTools, data);
            heightMapUpdater.addPointCloudToQueue(Triple.of(pointCloudData, zeroPose, gridCenter));
         }
      });
       */

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(new ROS2Helper(ros2Node));
      ros2PropertySetGroup.registerStoredPropertySet(HeightMapAPI.PARAMETERS, heightMapUpdater.getHeightMapParameters());
      ros2PropertySetGroup.registerStoredPropertySet(HeightMapAPI.FILTER_PARAMETERS, heightMapUpdater.getHeightMapFilterParameters());

      heightMapUpdater.setPublishFrequency(initialPublishFrequency);
      heightMapUpdater.setEnableUpdates(true);

      ros2Node.spin();
      executorService.scheduleAtFixedRate(this::update, 0, updateDTMillis, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      ros2PropertySetGroup.update();

      if (heightMapUpdater.updatesAreEnabled())
      {
         if (!updateThreadIsRunning.getAndSet(true))
         {
            heightMapUpdater.runFullUpdate(updateDTMillis);
            updateThreadIsRunning.set(false);
         }
      }
   }

   public void stop()
   {
      ros2Node.destroy();
      executorService.shutdown();
      perceptionMessageTools.destroy();
   }
}
