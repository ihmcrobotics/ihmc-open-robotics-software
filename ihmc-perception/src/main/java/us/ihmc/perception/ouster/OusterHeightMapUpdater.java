package us.ihmc.perception.ouster;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import org.apache.commons.lang3.tuple.Triple;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ihmcPerception.depthData.PointCloudData;
import us.ihmc.ihmcPerception.heightMap.HeightMapAPI;
import us.ihmc.ihmcPerception.heightMap.HeightMapUpdater;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.nio.FloatBuffer;
import java.time.Instant;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class OusterHeightMapUpdater
{
   private static final long updateDTMillis = 100;
   private static final int initialPublishFrequency = 5;

   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<HeightMapMessage> heightMapPublisher;
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean(false);
   private final HeightMapUpdater heightMapUpdater;

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService
         = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()),
                                                                 ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public OusterHeightMapUpdater(ROS2ControllerPublishSubscribeAPI ros2)
   {
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_height_map_publisher");
      heightMapPublisher = ROS2Tools.createPublisher(realtimeROS2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);
      ros2.subscribeViaCallback(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, this::consumeStateRequestMessage);
      ros2.subscribeToControllerViaCallback(HighLevelStateChangeStatusMessage.class, this::consumeStateChangedMessage);

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
      executorService.scheduleAtFixedRate(this::update, 0, updateDTMillis, TimeUnit.MILLISECONDS);
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

      // submitting the world frame for the sensor pose, as that's the frame the data is in.
      heightMapUpdater.addPointCloudToQueue(Triple.of(pointCloudData, new FramePose3D(ReferenceFrame.getWorldFrame()), gridCenter));
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
         heightMapUpdater.requestClear();
      else if (fromState != HighLevelControllerName.CUSTOM1 && toState == HighLevelControllerName.WALKING)
         heightMapUpdater.requestClear();
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
      executorService.shutdown();
   }

   public void destroy()
   {
      realtimeROS2Node.destroy();
   }
}
