package us.ihmc.perception;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.sensors.realsense.RealSenseImageSensor;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

/**
 * This class handles publishing the color and depth of the realsense. Its meant to be a standalone class that only
 * touches the realsense.
 */
public class StandAloneRealsenseProcess
{
   public static final String STAND_ALONE_REALSENSE_PROCESS = "StandAloneRealsenseProcess";

   private final ROS2DemandGraphNode realsenseDemandNode;
   private final ROS2DemandGraphNode realsensePublishDemandNode;

   private final RealSenseImageSensor d455Sensor;
   private final ImageSensorPublishThread d455PublishThread;

   private final RapidHeightMapThread rapidHeightMapThread;

   public StandAloneRealsenseProcess(ROS2Node ros2Node,
                                     ROS2Helper ros2Helper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RobotCollisionModel robotCollisionModel,
                                     HeightMapParameters heightMapParameters,
                                     DepthImageFilteringParameters depthImageFilteringParameters)
   {
      this(ros2Node, ros2Helper, syncedRobot, robotCollisionModel, heightMapParameters, depthImageFilteringParameters, null);
   }

   public StandAloneRealsenseProcess(ROS2Node ros2Node,
                                     ROS2Helper ros2Helper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RobotCollisionModel robotCollisionModel,
                                     HeightMapParameters heightMapParameters,
                                     DepthImageFilteringParameters depthImageFilteringParameters,
                                     ControllerFootstepQueueMonitor controllerFootstepQueueMonitor)
   {

      realsensePublishDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE_PUBLICATION);
      ROS2DemandGraphNode heightMapDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_HEIGHT_MAP);

      realsenseDemandNode = new ROS2DemandGraphNode(ros2Node, PerceptionAPI.REQUEST_REALSENSE);
      realsenseDemandNode.addDependents(realsensePublishDemandNode, heightMapDemandNode);

      d455Sensor = new RealSenseImageSensor(RealSenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ);

      ROS2TunedRigidBodyTransform realsenseTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                                                    PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                                                                    syncedRobot.getRobotModel()
                                                                                                               .getSensorInformation()
                                                                                                               .getSteppingCameraTransform());

      // We create a ThreadFactory here so that when profiling the thread, there is a user-friendly name to identify it with
      ThreadFactory threadFactory = new ThreadFactoryBuilder().setNameFormat(STAND_ALONE_REALSENSE_PROCESS).build();
      ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1, threadFactory);
      scheduler.scheduleAtFixedRate(realsenseTunableTransform::update, 0, 33, TimeUnit.MILLISECONDS);

      d455Sensor.setSensorFrame(syncedRobot.getReferenceFrames().getSteppingCameraFrame());
      loopOnDemand(d455Sensor.getGrabThread(), realsenseDemandNode);

      d455PublishThread = new ImageSensorPublishThread(ros2Node, d455Sensor);
      d455PublishThread.addTopic(PerceptionAPI.SRT_REALSENSE_COLOR_STREAM_STATUS, RealSenseImageSensor.COLOR_IMAGE_KEY);
      d455PublishThread.addTopic(PerceptionAPI.D455_DEPTH_IMAGE, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      loopOnDemand(d455PublishThread, realsensePublishDemandNode);

      BlockingQueue<RawImage> rawImageCollection = new LinkedBlockingQueue<>(10);
      d455Sensor.registerImageCollector(rawImageCollection, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      rapidHeightMapThread = new RapidHeightMapThread(ros2Helper.getROS2Node(),
                                                      syncedRobot,
                                                      robotCollisionModel,
                                                      rawImageCollection,
                                                      controllerFootstepQueueMonitor,
                                                      heightMapParameters,
                                                      depthImageFilteringParameters);
      rapidHeightMapThread.startRepeating();
   }

   public HeightMapData getLatestHeightMapData()
   {
      return rapidHeightMapThread.getLatestHeightMapData();
   }

   public void destroy()
   {
      rapidHeightMapThread.blockingKill();
      realsenseDemandNode.destroy();
      realsensePublishDemandNode.destroy();
      d455Sensor.close();
      d455PublishThread.blockingKill();
   }

   private static void loopOnDemand(RepeatingTaskThread loopThread, ROS2DemandGraphNode demandNode)
   {
      if (!loopThread.isAlive())
         loopThread.start();

      if (demandNode.isDemanded())
         loopThread.startRepeating();

      demandNode.addDemandChangedCallback(loopThread::setRepeating);
   }
}
