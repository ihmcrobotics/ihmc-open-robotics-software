package us.ihmc.avatar.networkProcessor.lidarScanPublisher;

import java.net.URI;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.SimulatedLidarScanPacket;
import gnu.trove.list.array.TFloatArrayList;
import scan_to_cloud.PointCloud2WithSource;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.CollidingScanPointFilter;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.RangeScanPointFilter;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

public class LidarScanPublisher
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int DEFAULT_MAX_NUMBER_OF_POINTS = 5000;

   private final String name = getClass().getSimpleName();
   private final ExceptionHandlingThreadScheduler executorService = new ExceptionHandlingThreadScheduler(name, DefaultExceptionHandler.PRINT_STACKTRACE);
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<PointCloudData> rosPointCloud2ToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullRobotModel fullRobotModel;
   private final ReferenceFrame lidarSensorFrame;
   private ReferenceFrame scanPointsFrame = worldFrame;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private RobotROSClockCalculator rosClockCalculator = null;

   private final IHMCROS2Publisher<LidarScanMessage> lidarScanPublisher;

   private int maximumNumberOfPoints = DEFAULT_MAX_NUMBER_OF_POINTS;
   private RangeScanPointFilter rangeFilter = null;
   private ShadowScanPointFilter shadowFilter = null;
   private CollidingScanPointFilter collisionFilter = null;
   private final ScanPointFilterList activeFilters = new ScanPointFilterList();

   private long publisherPeriodInMillisecond = 1L;

   public LidarScanPublisher(String lidarName, FullRobotModelFactory modelFactory, ROS2NodeInterface ros2Node)
   {
      this(modelFactory, defaultSensorFrameFactory(lidarName), ros2Node);
   }

   public LidarScanPublisher(FullRobotModelFactory modelFactory, SensorFrameFactory sensorFrameFactory, ROS2NodeInterface ros2Node)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), sensorFrameFactory, ros2Node);
   }
   public LidarScanPublisher(String robotName,
                              FullRobotModel fullRobotModel,
                              SensorFrameFactory sensorFrameFactory,
                              ROS2NodeInterface ros2Node)
   {
      this.robotName = robotName;
      this.fullRobotModel = fullRobotModel;
      lidarSensorFrame = sensorFrameFactory.setupSensorFrame(fullRobotModel);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      lidarScanPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
   }

   public void setMaximumNumberOfPoints(int maximumNumberOfPoints)
   {
      this.maximumNumberOfPoints = maximumNumberOfPoints;
   }

   public void setPublisherPeriodInMillisecond(long publisherPeriodInMillisecond)
   {
      this.publisherPeriodInMillisecond = publisherPeriodInMillisecond;

      if (publisherTask != null)
      {
         publisherTask.cancel(false);
         start();
      }
   }

   public void start()
   {
      publisherTask = executorService.schedule(this::readAndPublishInternal, publisherPeriodInMillisecond, TimeUnit.MILLISECONDS);
   }

   public void shutdown()
   {
      publisherTask.cancel(false);
      executorService.shutdown();
   }

   public void setScanFrameToLidarSensorFrame()
   {
      scanPointsFrame = lidarSensorFrame;
   }

   public void setRangeFilter(double minRange, double maxRange)
   {
      if (rangeFilter == null)
      {
         rangeFilter = new RangeScanPointFilter();
         activeFilters.addFilter(rangeFilter);
      }

      rangeFilter.setMinRange(minRange);
      rangeFilter.setMaxRange(maxRange);
   }

   public void setShadowFilter()
   {
      setShadowFilter(ShadowScanPointFilter.DEFAULT_SHADOW_ANGLE_THRESHOLD);
   }

   public void setShadowFilter(double angleThreshold)
   {
      if (shadowFilter == null)
      {
         shadowFilter = new ShadowScanPointFilter();
         activeFilters.addFilter(shadowFilter);
      }

      shadowFilter.setShadowAngleThreshold(angleThreshold);
   }

   public void setSelfCollisionFilter(CollisionBoxProvider collisionBoxProvider)
   {
      if (collisionFilter != null)
         return;
      collisionFilter = new CollidingScanPointFilter(new CollisionShapeTester(fullRobotModel, collisionBoxProvider));
      activeFilters.addFilter(collisionFilter);
   }

   public void receiveLidarFromROS(String lidarScanROSTopic, URI rosCoreURI)
   {
      String graphName = robotName + "/" + name;
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, graphName, true);
      receiveLidarFromROS(lidarScanROSTopic, rosMainNode);
   }

   public void receiveLidarFromROS(String lidarScanROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(lidarScanROSTopic, createROSPointCloud2Subscriber());
   }

   /**
    * This is to subscribe to non-standard topic's message: PointCloud2WithSource.
    */
   public void receiveLidarFromROSAsPointCloud2WithSource(String lidarScanROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(lidarScanROSTopic, createROSPointCloud2WithSourceSubscriber());
   }

   public void receiveLidarFromSCS(ObjectCommunicator scsSensorsCommunicator)
   {
      scsSensorsCommunicator.attachListener(SimulatedLidarScanPacket.class, createSimulatedLidarScanPacketConsumer());
   }

   public void setScanFrameToWorldFrame()
   {
      scanPointsFrame = worldFrame;
   }

   public void setROSClockCalculator(RobotROSClockCalculator rosClockCalculator)
   {
      this.rosClockCalculator = rosClockCalculator;
   }

   private RosPointCloudSubscriber createROSPointCloud2Subscriber()
   {
      return new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            rosPointCloud2ToPublish.set(new PointCloudData(pointCloud, maximumNumberOfPoints, false));
         }
      };
   }

   private RosTopicSubscriberInterface<PointCloud2WithSource> createROSPointCloud2WithSourceSubscriber()
   {
      return new AbstractRosTopicSubscriber<PointCloud2WithSource>(PointCloud2WithSource._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2WithSource pointCloud)
         {
            rosPointCloud2ToPublish.set(new PointCloudData(pointCloud.getCloud(), maximumNumberOfPoints, false));
         }
      };
   }

   private ObjectConsumer<SimulatedLidarScanPacket> createSimulatedLidarScanPacketConsumer()
   {
      return new ObjectConsumer<SimulatedLidarScanPacket>()
      {
         private final RigidBodyTransform identityTransform = new RigidBodyTransform();

         @Override
         public void consumeObject(SimulatedLidarScanPacket packet)
         {
            LidarScanParameters lidarScanParameters = MessageTools.toLidarScanParameters(packet.getLidarScanParameters());
            TFloatArrayList ranges = packet.getRanges();
            int sensorId = packet.getSensorId();
            LidarScan scan = new LidarScan(lidarScanParameters, ranges.toArray(), sensorId);
            // Set the world transforms to nothing, so points are in lidar scan frame
            scan.setWorldTransforms(identityTransform, identityTransform);
            Point3D[] scanPoints = scan.toPointArray();
            long timestamp = packet.getLidarScanParameters().getTimestamp();

            rosPointCloud2ToPublish.set(new PointCloudData(timestamp, scanPoints, null));
         }
      };
   }

   public void updateScanData(PointCloudData scanDataToPublish)
   {
      this.rosPointCloud2ToPublish.set(scanDataToPublish);
   }

   // Temporary variables used to find shadows
   private final Pose3D sensorPose = new Pose3D();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public LidarScanMessage readAndPublish()
   {
      if (publisherTask != null)
         throw new RuntimeException("The publisher is running using its own thread, cannot manually update it.");

      return readAndPublishInternal();
   }

   private LidarScanMessage readAndPublishInternal()
   {
      PointCloudData pointCloudData = rosPointCloud2ToPublish.getAndSet(null);

      if (pointCloudData == null)
         return null;

      long robotTimestamp;

      if (rosClockCalculator == null)
      {
         robotTimestamp = pointCloudData.getTimestamp();
         robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
      }
      else
      {
         long rosTimestamp = pointCloudData.getTimestamp();
         robotTimestamp = rosClockCalculator.computeRobotMonotonicTime(rosTimestamp);
         if (robotTimestamp == -1L)
            return null;
         boolean waitForTimestamp = true;
         if (robotConfigurationDataBuffer.getNewestTimestamp() == -1)
            return null;

         boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;

         if (!success)
            return null;
      }

      if (!scanPointsFrame.isWorldFrame())
      {
         scanPointsFrame.getTransformToDesiredFrame(transformToWorld, worldFrame);
         pointCloudData.applyTransform(transformToWorld);
      }

      sensorPose.set(lidarSensorFrame.getTransformToRoot());

      if (shadowFilter != null)
         shadowFilter.set(sensorPose.getPosition(), pointCloudData);
      if (collisionFilter != null)
         collisionFilter.update();
      if (rangeFilter != null)
         rangeFilter.setSensorPosition(sensorPose.getPosition());

      LidarScanMessage message = pointCloudData.toLidarScanMessage(activeFilters);
      message.getLidarPosition().set(sensorPose.getPosition());
      message.getLidarOrientation().set(sensorPose.getOrientation());

      lidarScanPublisher.publish(message);

      return message;
   }

   public static interface SensorFrameFactory
   {
      ReferenceFrame setupSensorFrame(FullRobotModel fullRobotModel);
   }

   public static SensorFrameFactory defaultSensorFrameFactory(String lidarName)
   {
      return fullRobotModel -> ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("lidarSensorFrame",
                                                                                                 fullRobotModel.getLidarBaseFrame(lidarName),
                                                                                                 fullRobotModel.getLidarBaseToSensorTransform(lidarName));
   }
}
