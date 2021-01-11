package us.ihmc.avatar.networkProcessor.trackingCameraPublisher;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StampedPosePacket;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Vector3;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosNavMsgsOdometrySubscriber;

public class TrackingCameraBridge
{
   private static final boolean Debug = false;

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(name));
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<TrackingCameraData> trackingCameraDataToPublish = new AtomicReference<>(null);
   private final AtomicReference<StampedPosePacket> stampedPosePacketToPublish = new AtomicReference<>(null);

   private final FullRobotModel fullRobotModel;

   private SensorFrameInitializationTransformer sensorFrameInitializationTransformer = null;
   private final RigidBodyTransform initialTransformToWorld = new RigidBodyTransform();
   private boolean initialized = false;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private RobotROSClockCalculator rosClockCalculator = null;

   private final Consumer<StampedPosePacket> stampedPosePacketPublisher;

   public TrackingCameraBridge(FullRobotModelFactory modelFactory, ROS2NodeInterface ros2Node)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node);
   }

   public TrackingCameraBridge(String robotName, FullRobotModel fullRobotModel, ROS2NodeInterface ros2Node)
   {
      this.fullRobotModel = fullRobotModel;

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                                    s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      stampedPosePacketPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.T265_POSE)::publish;
   }

   public TrackingCameraBridge(String robotName, FullRobotModel fullRobotModel, RealtimeROS2Node realtimeROS2Node)
   {
      this.fullRobotModel = fullRobotModel;

      ROS2Tools.createCallbackSubscription(realtimeROS2Node,
                                           ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                           s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
      stampedPosePacketPublisher = ROS2Tools.createPublisher(realtimeROS2Node, ROS2Tools.T265_POSE)::publish;
   }

   public void start()
   {
      publisherTask = executorService.scheduleAtFixedRate(this::readAndPublishInternal, 0L, 1L, TimeUnit.MILLISECONDS);
   }

   public void shutdown()
   {
      publisherTask.cancel(false);
      executorService.shutdownNow();
   }

   public void receiveTrackingCameraDataFromROS1(String trackingCameraDataROSTopic, RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(trackingCameraDataROSTopic, createNavigationMessageSubscriber());
   }

   public void setROSClockCalculator(RobotROSClockCalculator rosClockCalculator)
   {
      this.rosClockCalculator = rosClockCalculator;
   }

   public void setCustomInitializationTransformer(SensorFrameInitializationTransformer transformer)
   {
      sensorFrameInitializationTransformer = transformer;
   }

   private RosNavMsgsOdometrySubscriber createNavigationMessageSubscriber()
   {
      return new RosNavMsgsOdometrySubscriber()
      {
         @Override
         public void onNewMessage(nav_msgs.Odometry message)
         {
            long timeStamp = message.getHeader().getStamp().totalNsecs();

            if (Debug)
               System.out.println("Odometry timeStamp " + timeStamp);

            Pose pose = message.getPose().getPose();
            Vector3 linearVelocity = message.getTwist().getTwist().getLinear();
            Vector3 angularVelocity = message.getTwist().getTwist().getAngular();
            TrackingCameraData trackingCameraData = new TrackingCameraData();
            trackingCameraData.setTimeStamp(timeStamp);
            trackingCameraData.setConfidence(1.0); // TODO: add confidence factor on Odometry.
            trackingCameraData.setPosition(pose.getPosition());
            trackingCameraData.setOrientation(pose.getOrientation());
            trackingCameraData.setLinearVelocity(linearVelocity);
            trackingCameraData.setAngularVelocity(angularVelocity);

            if (Debug)
               System.out.println("message.getPose().getPose() " + message.getPose().getPose().getPosition().getX());

            trackingCameraDataToPublish.set(trackingCameraData);
         }

         @Override
         protected void newPose(String frameID, TimeStampedTransform3D transform)
         {

         }
      };
   }

   public void readAndPublish()
   {
      if (publisherTask != null)
         throw new RuntimeException("The publisher is running using its own thread, cannot manually update it.");

      readAndPublishInternal();
   }

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
      TrackingCameraData dataToPublish = trackingCameraDataToPublish.getAndSet(null);

      if (dataToPublish == null)
         return;

      long robotTimestamp;

      if (rosClockCalculator == null)
      {
         robotTimestamp = dataToPublish.getTimeStamp();
         robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
      }
      else
      {
         long rosTimestamp = dataToPublish.getTimeStamp();
         robotTimestamp = rosClockCalculator.computeRobotMonotonicTime(rosTimestamp);
         boolean waitForTimestamp = true;
         if (robotConfigurationDataBuffer.getNewestTimestamp() == -1)
            return;

         boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;

         if (!success)
            return;
      }

      if (!initialized && sensorFrameInitializationTransformer != null)
      {
         initialized = true;
         sensorFrameInitializationTransformer.computeTransformToWorld(fullRobotModel, initialTransformToWorld);
         computeTrackingCameraInitialTransform(initialTransformToWorld, dataToPublish);

         return;
      }
      dataToPublish.applyTransform(initialTransformToWorld);

      StampedPosePacket message = dataToPublish.toPacket();
      stampedPosePacketToPublish.set(message);

      if (Debug)
         System.out.println("Publishing tracking camera data.");

      stampedPosePacketPublisher.accept(message);
   }
   
   public StampedPosePacket pollNewData()
   {
      return stampedPosePacketToPublish.getAndSet(null);
   }

   private class TrackingCameraData
   {
      long timeStamp;
      double confidence;
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      Vector3D linearVelocity = new Vector3D();
      Vector3D angularVelocity = new Vector3D();

      public void setTimeStamp(long timeStamp)
      {
         this.timeStamp = timeStamp;
      }

      public void setConfidence(double confidence)
      {
         this.confidence = confidence;
      }

      public void setOrientation(geometry_msgs.Quaternion quaternion)
      {
         this.orientation.set(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());
      }

      public void setPosition(Point position)
      {
         this.position.set(position.getX(), position.getY(), position.getZ());
      }

      public void setLinearVelocity(Vector3 linearVelocity)
      {
         this.linearVelocity.set(linearVelocity.getX(), linearVelocity.getY(), linearVelocity.getZ());
      }

      public void setAngularVelocity(Vector3 angularVelocity)
      {
         this.angularVelocity.set(angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ());
      }

      public void applyTransform(RigidBodyTransform transformToWorld)
      {
         transformToWorld.transform(position);
         transformToWorld.transform(orientation);
         transformToWorld.transform(linearVelocity);
         transformToWorld.transform(angularVelocity);
      }

      public long getTimeStamp()
      {
         return timeStamp;
      }

      public StampedPosePacket toPacket()
      {
         StampedPosePacket message = new StampedPosePacket();

         message.getPose().getPosition().set(position);
         message.getPose().getOrientation().set(orientation);
         message.getTwist().getLinear().set(linearVelocity);
         message.getTwist().getAngular().set(angularVelocity);
         message.setTimestamp(timeStamp);
         message.setConfidenceFactor(confidence);

         return message;
      }
   }

   /**
    * Tracking camera(T265) is able to detect horizontal plane.
    * The initialTransformToWorld will be packed with following equation.
    * T_w_p: transform world to tracking camera frame.
    * T_i : transform generated by the initial tracking camera data.
    * 
    * T_to_pack = T_w_p * inv(T_i));
    */
   private void computeTrackingCameraInitialTransform(RigidBodyTransform initialTransformToWorld, TrackingCameraData initialTrackingCameraData)
   {
      initialTransformToWorld.multiplyInvertOther(new RigidBodyTransform(initialTrackingCameraData.orientation, initialTrackingCameraData.position));
   }

   public static interface SensorFrameInitializationTransformer
   {
      public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack);
   }
}
