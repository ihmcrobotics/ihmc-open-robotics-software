package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.net.URI;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessagePubSubType;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.lidarScanPublisher.ScanPointFilterList;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class StereoVisionPointCloudPublisher
{
   private static final boolean Debug = false;

   private static final int DEFAULT_MAX_NUMBER_OF_POINTS = 50000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<PointCloudData> rosPointCloud2ToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullRobotModel fullRobotModel;
   private ReferenceFrame stereoVisionPointsFrame = worldFrame;
   private StereoVisionWorldTransformCalculator stereoVisionTransformer = null;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private RobotROSClockCalculator rosClockCalculator = null;

   private final Consumer<StereoVisionPointCloudMessage> pointcloudPublisher;

   private int maximumNumberOfPoints = DEFAULT_MAX_NUMBER_OF_POINTS;
   private RangeScanPointFilter rangeFilter = null;
   private CollidingScanPointFilter collisionFilter;
   private final ScanPointFilterList activeFilters = new ScanPointFilterList();

   /**
    * units of velocities are meter/sec and rad/sec.
    */
   private long previousTimeStamp = 0;
   private final Point3D previousSensorPosition = new Point3D();
   private final Quaternion previousSensorOrientation = new Quaternion();
   private final AtomicBoolean enableFilter = new AtomicBoolean(false);
   private final AtomicDouble linearVelocityThreshold = new AtomicDouble(Double.MAX_VALUE);
   private final AtomicDouble angularVelocityThreshold = new AtomicDouble(Double.MAX_VALUE);

   private long publisherPeriodInMillisecond = 200L;
   private double minimumResolution = 0.005;

   public StereoVisionPointCloudPublisher(FullRobotModelFactory modelFactory, ROS2NodeInterface ros2Node, ROS2Topic<StereoVisionPointCloudMessage> topic)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node, topic);
   }

   public StereoVisionPointCloudPublisher(String robotName,
                                          FullRobotModel fullRobotModel,
                                          ROS2NodeInterface ros2Node,
                                          ROS2Topic<StereoVisionPointCloudMessage> topic)
   {
      this.robotName = robotName;
      this.fullRobotModel = fullRobotModel;

         ROS2Tools.createCallbackSubscription(ros2Node,
                                              ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      LogTools.info("Creating stereo point cloud publisher. Topic name: {}", topic.getName());
      pointcloudPublisher = ROS2Tools.createPublisher(ros2Node, topic)::publish;
   }

   public StereoVisionPointCloudPublisher(String robotName,
                                          FullRobotModel fullRobotModel,
                                          RealtimeROS2Node realtimeROS2Node,
                                          ROS2Topic<StereoVisionPointCloudMessage> topic)
   {
      this.robotName = robotName;
      this.fullRobotModel = fullRobotModel;

      ROS2Tools.createCallbackSubscription(realtimeROS2Node,
                                           ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      LogTools.info("Creating stereo point cloud publisher. Topic name: {}", topic.getName());
      pointcloudPublisher = ROS2Tools.createPublisher(realtimeROS2Node, topic)::publish;
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
      publisherTask = executorService.scheduleAtFixedRate(this::readAndPublishInternal, 0L, publisherPeriodInMillisecond, TimeUnit.MILLISECONDS);
   }

   public void shutdown()
   {
      publisherTask.cancel(false);
      executorService.shutdownNow();
   }

   /**
    * Sets the smallest value in meter for the resolution when compressing the pointcloud data.
    * <p>
    * Smaller means more accurate, but also more expensive bandwidth-wise.
    * </p>
    * 
    * @param minimumResolution the value in meter for the minimum resolution, default is 0.005.
    */
   public void setMinimumResolution(double minimumResolution)
   {
      this.minimumResolution = minimumResolution;
   }

   public void setSelfCollisionFilter(CollisionBoxProvider collisionBoxProvider)
   {
      if (collisionFilter != null)
         return;
      collisionFilter = new CollidingScanPointFilter(new CollisionShapeTester(fullRobotModel, collisionBoxProvider));
      activeFilters.addFilter(collisionFilter);
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

   public void receiveStereoPointCloudFromROS1(String stereoPointCloudROSTopic, URI rosCoreURI)
   {
      String graphName = robotName + "/" + name;
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, graphName, true);
      receiveStereoPointCloudFromROS1(stereoPointCloudROSTopic, rosMainNode);
   }

   public void receiveStereoPointCloudFromROS1(String stereoPointCloudROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(stereoPointCloudROSTopic, createROSPointCloud2Subscriber());
   }

   public void setROSClockCalculator(RobotROSClockCalculator rosClockCalculator)
   {
      this.rosClockCalculator = rosClockCalculator;
   }

   public void setCustomStereoVisionTransformer(StereoVisionWorldTransformCalculator transformer)
   {
      stereoVisionTransformer = transformer;
   }

   private RosPointCloudSubscriber createROSPointCloud2Subscriber()
   {
      return new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            rosPointCloud2ToPublish.set(new PointCloudData(pointCloud, maximumNumberOfPoints, true));

            if (Debug)
               System.out.println("Receiving point cloud, n points: " + pointCloud.getHeight() * pointCloud.getWidth());
         }
      };
   }

   public void updateScanData(PointCloudData scanDataToPublish)
   {
      this.rosPointCloud2ToPublish.set(scanDataToPublish);
   }

   public void readAndPublish()
   {
      if (publisherTask != null)
         throw new RuntimeException("The publisher is running using its own thread, cannot manually update it.");

      readAndPublishInternal();
   }

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final Pose3D sensorPose = new Pose3D();

   private void readAndPublishInternal()
   {
      try
      {
         transformDataAndPublish();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         executorService.shutdown();
      }
   }

   private void transformDataAndPublish()
   {
      PointCloudData pointCloudData = rosPointCloud2ToPublish.getAndSet(null);

      if (pointCloudData == null)
         return;

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
         boolean waitForTimestamp = true;
         if (robotConfigurationDataBuffer.getNewestTimestamp() == -1)
            return;

         boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;

         if (!success)
            return;
      }

      if (stereoVisionTransformer != null)
      {
         stereoVisionTransformer.computeTransformToWorld(fullRobotModel, transformToWorld, sensorPose);
         pointCloudData.applyTransform(transformToWorld);
      }
      else
      {
         if (!stereoVisionPointsFrame.isWorldFrame())
         {
            stereoVisionPointsFrame.getTransformToDesiredFrame(transformToWorld, worldFrame);
            pointCloudData.applyTransform(transformToWorld);
         }

         fullRobotModel.getHeadBaseFrame().getTransformToDesiredFrame(transformToWorld, worldFrame);
         sensorPose.set(transformToWorld);
      }

      if (enableFilter.get())
      {
         double timeDiff = Conversions.nanosecondsToSeconds(robotTimestamp - previousTimeStamp);
         double linearVelocity = sensorPose.getPosition().distance(previousSensorPosition) / timeDiff;
         double angularVelocity = sensorPose.getOrientation().distance(previousSensorOrientation) / timeDiff;

         previousTimeStamp = robotTimestamp;
         previousSensorPosition.set(sensorPose.getPosition());
         previousSensorOrientation.set(sensorPose.getOrientation());

         if (linearVelocity > linearVelocityThreshold.get() || angularVelocity > angularVelocityThreshold.get())
            return;
      }

      if (collisionFilter != null)
         collisionFilter.update();
      if (rangeFilter != null)
         rangeFilter.setSensorPosition(sensorPose.getPosition());

      long startTime = System.nanoTime();
      StereoVisionPointCloudMessage message = pointCloudData.toStereoVisionPointCloudMessage(minimumResolution, activeFilters);

      if (message == null)
         return; // TODO Sometimes the LZ4 compression fails. Need to figure it out, for now just giving up.

      message.getSensorPosition().set(sensorPose.getPosition());
      message.getSensorOrientation().set(sensorPose.getOrientation());
      long endTime = System.nanoTime();

      if (Debug)
         System.out.println("Publishing stereo data, number of points: " + (message.getPointCloud().size() / 3) + ", packet size in kilobytes: "
               + (StereoVisionPointCloudMessagePubSubType.getCdrSerializedSize(message) / 1000) + ", compression time in milliseconds: "
               + Conversions.nanosecondsToMilliseconds(endTime - startTime));
      pointcloudPublisher.accept(message);
   }

   public void enableFilter(boolean enable)
   {
      enableFilter.set(enable);
   }

   public void setFilterThreshold(double linearVelocityThreshold, double angularVelocityThreshold)
   {
      this.linearVelocityThreshold.set(linearVelocityThreshold);
      this.angularVelocityThreshold.set(angularVelocityThreshold);
   }

   public static interface StereoVisionWorldTransformCalculator
   {
      public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack, Pose3DBasics sensorPoseToPack);
   }
}
