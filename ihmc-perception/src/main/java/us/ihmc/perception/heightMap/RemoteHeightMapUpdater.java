package us.ihmc.perception.heightMap;

import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.perception.gpuHeightMap.HeightMapKernel;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParametersBasics;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RemoteHeightMapUpdater
{
   private static final long updateDTMillis = 100;
   private static final int initialPublishFrequency = 5;

   private final RealtimeROS2Node ros2Node;
   private static final FramePose3D zeroPose = new FramePose3D();
   private final HeightMapKernel heightMapKernel = new HeightMapKernel();
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean(false);
   private final HeightMapUpdater heightMapUpdater;

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final AtomicReference<WalkingStatus> currentWalkingStatus = new AtomicReference<>();

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()),
                                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public RemoteHeightMapUpdater(String robotName, Supplier<ReferenceFrame> groundFrameProvider, RealtimeROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;

      ROS2PublisherBasics<HeightMapMessage> heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_OUTPUT);
      ROS2Tools.createCallbackSubscription(ros2Node, PerceptionAPI.HEIGHT_MAP_STATE_REQUEST, m -> consumeStateRequestMessage(m.readNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotName), m -> consumeWalkingStatusMessage(m.readNextData()));

      heightMapUpdater = new HeightMapUpdater();
      heightMapUpdater.attachHeightMapConsumer(heightMapPublisher::publish);

      ROS2Tools.createCallbackSubscription(ros2Node, PerceptionAPI.OUSTER_LIDAR_SCAN, new NewMessageListener<LidarScanMessage>()
      {
         @Override
         public void onNewDataMessage(Subscriber<LidarScanMessage> subscriber)
         {
            ReferenceFrame groundFrame = groundFrameProvider.get();
            double groundHeight = groundFrame.isRootFrame() ? 0.0 : groundFrame.getTransformToRoot().getTranslationZ();

            LidarScanMessage data = subscriber.readNextData();
            //            FramePose3D ousterPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());
            Point3D gridCenter = new Point3D(data.getLidarPosition().getX(), data.getLidarPosition().getY(), groundHeight);
            PointCloudData pointCloudData = new PointCloudData(data);
            FramePose3D sensorPose = new FramePose3D(ReferenceFrame.getWorldFrame(), data.getLidarPosition(), data.getLidarOrientation());

            HeightMapInputData inputData = new HeightMapInputData();
            inputData.pointCloud = pointCloudData;
            inputData.sensorPose = sensorPose;
            inputData.gridCenter = gridCenter;
            if (currentWalkingStatus.get() == WalkingStatus.STARTED)
            {
               inputData.verticalMeasurementVariance = heightMapUpdater.getHeightMapParameters().getSensorVarianceWhenMoving();
            }
            else
            {
               inputData.verticalMeasurementVariance = heightMapUpdater.getHeightMapParameters().getSensorVarianceWhenStanding();
            }

            heightMapUpdater.addPointCloudToQueue(inputData);
         }
      });

      /*
      ROS2Tools.createCallbackSubscription(ros2Node, PerceptionAPI.OUSTER_DEPTH_IMAGE, ROS2QosProfile.BEST_EFFORT(), new NewMessageListener<ImageMessage>()
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
   }

   private void consumeWalkingStatusMessage(WalkingStatusMessage message)
   {
      currentWalkingStatus.set(WalkingStatus.fromByte(message.getWalkingStatus()));
   }

   public void start()
   {
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

   public HeightMapParametersBasics getParameters()
   {
      return heightMapUpdater.getHeightMapParameters();
   }

   public HeightMapData getLatestHeightMap()
   {
      return heightMapUpdater.getLatestHeightMap();
   }

   public void attachHeightMapConsumer(Consumer<HeightMapMessage> heightMapMessageConsumer)
   {
      heightMapUpdater.attachHeightMapConsumer(heightMapMessageConsumer);
   }

   public void consumeStateRequestMessage(HeightMapStateRequestMessage message)
   {
      if (message.getRequestPause())
         heightMapUpdater.requestPause();
      else if (message.getRequestResume())
         heightMapUpdater.requestResume();

      if (message.getRequestClear())
         heightMapUpdater.requestClear();
   }

   public void stop()
   {
      ros2Node.destroy();
      executorService.shutdown();
      heightMapKernel.destroy();
   }
}
