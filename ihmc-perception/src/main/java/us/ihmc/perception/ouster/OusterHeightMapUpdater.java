package us.ihmc.perception.ouster;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.perception.heightMap.HeightMapAPI;
import us.ihmc.perception.heightMap.HeightMapInputData;
import us.ihmc.perception.heightMap.HeightMapUpdater;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.nio.FloatBuffer;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class OusterHeightMapUpdater
{
   private static final long updateDTMillis = 100;
   private static final double updateDTSeconds = (double) updateDTMillis / 1000;
   private static final int initialPublishFrequency = 5;

   private final RealtimeROS2Node realtimeROS2Node;
   private final ROS2PublisherBasics<HeightMapMessage> heightMapPublisher;
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean(false);
   private final AtomicReference<WalkingStatus> currentWalkingStatus = new AtomicReference<>();
   private final HeightMapUpdater heightMapUpdater;

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final PausablePeriodicThread updateThread = new PausablePeriodicThread("OusterHeightMapUpdater", updateDTSeconds, this::update);

   public OusterHeightMapUpdater(ROS2ControllerPublishSubscribeAPI ros2)
   {
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_height_map_publisher");
      heightMapPublisher = realtimeROS2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_OUTPUT);
      ros2.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_STATE_REQUEST, this::consumeStateRequestMessage);
      ros2.subscribeToControllerViaCallback(HighLevelStateChangeStatusMessage.class, this::consumeStateChangedMessage);
      ros2.subscribeToControllerViaCallback(WalkingStatusMessage.class, this::consumeWalkingStatusMessage);

      heightMapUpdater = new HeightMapUpdater();
      heightMapUpdater.attachHeightMapConsumer(heightMapPublisher::publish);

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2);
      ros2PropertySetGroup.registerStoredPropertySet(HeightMapAPI.PARAMETERS, heightMapUpdater.getHeightMapParameters());
      ros2PropertySetGroup.registerStoredPropertySet(HeightMapAPI.FILTER_PARAMETERS, heightMapUpdater.getHeightMapFilterParameters());

      heightMapUpdater.setPublishFrequency(initialPublishFrequency);
      heightMapUpdater.setEnableUpdates(true);

      realtimeROS2Node.spin();
   }

   public void start()
   {
      updateThread.start();
   }

   public void attachHeightMapConsumer(Consumer<HeightMapMessage> heightMapConsumer)
   {
      heightMapUpdater.attachHeightMapConsumer(heightMapConsumer);
   }

   public void updateWithDataBuffer(ReferenceFrame groundFrame,
                                    ReferenceFrame sensorFrame,
                                    FloatBuffer pointCloudInWorldFrame,
                                    int numberOfPoints,
                                    Instant instant)
   {
      double groundHeight = groundFrame.getTransformToRoot().getTranslationZ();

      FramePose3D sensorPose = new FramePose3D(sensorFrame);
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D gridCenter = new Point3D(sensorPose.getX(), sensorPose.getY(), groundHeight);
      PointCloudData pointCloudData = new PointCloudData(instant, numberOfPoints, pointCloudInWorldFrame);
      HeightMapInputData inputData = new HeightMapInputData();
      inputData.pointCloud = pointCloudData;
      inputData.gridCenter = gridCenter;
      // submitting the world frame for the sensor pose, as that's the frame the data is in.
      inputData.sensorPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      // TODO add variance
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

   public void consumeStateRequestMessage(HeightMapStateRequestMessage message)
   {
      if (message.getRequestPause())
         heightMapUpdater.requestPause();
      else if (message.getRequestResume())
         heightMapUpdater.requestResume();

      if (message.getRequestClear())
         heightMapUpdater.requestClear();
   }

   public void consumeStateChangedMessage(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName fromState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      HighLevelControllerName toState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      if (fromState == HighLevelControllerName.WALKING && toState != HighLevelControllerName.CUSTOM1)
      {
         heightMapUpdater.requestClear();
         currentWalkingStatus.set(WalkingStatus.COMPLETED);
      }
      else if (fromState != HighLevelControllerName.CUSTOM1 && toState == HighLevelControllerName.WALKING)
      {
         heightMapUpdater.requestClear();
         currentWalkingStatus.set(WalkingStatus.COMPLETED);
      }
   }

   public void consumeWalkingStatusMessage(WalkingStatusMessage message)
   {
      currentWalkingStatus.set(WalkingStatus.fromByte(message.getWalkingStatus()));
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
      updateThread.stop();
   }

   public void destroy()
   {
      realtimeROS2Node.destroy();
   }
}
