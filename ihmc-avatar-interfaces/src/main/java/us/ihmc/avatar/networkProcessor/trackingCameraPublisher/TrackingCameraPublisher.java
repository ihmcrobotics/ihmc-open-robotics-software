package us.ihmc.avatar.networkProcessor.trackingCameraPublisher;

import java.net.URI;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StampedPosePacket;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Vector3;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosNavMsgsOdometrySubscriber;

public class TrackingCameraPublisher
{
   private static final boolean Debug = false;

   private static final Class<StampedPosePacket> messageTypeToPublish = StampedPosePacket.class;

   private final String name = getClass().getSimpleName();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   private ScheduledFuture<?> publisherTask;

   private final AtomicReference<TrackingCameraData> trackingCameraDataToPublish = new AtomicReference<>(null);

   private final String robotName;
   private final FullRobotModel fullRobotModel;
   private SensorFrameInitializationTransformer sensorFrameInitializationTransformer = null;
   private boolean isInitialized = false;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private RobotROSClockCalculator rosClockCalculator = null;

   private final IHMCROS2Publisher<StampedPosePacket> stampedPosePacketPublisher;
   private final IHMCRealtimeROS2Publisher<StampedPosePacket> stampedPosePacketRealtimePublisher;

   public TrackingCameraPublisher(FullRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node, null, robotConfigurationDataTopicName,
           ROS2Tools.getDefaultTopicNameGenerator());
   }

   public TrackingCameraPublisher(FullRobotModelFactory modelFactory, Ros2Node ros2Node, String robotConfigurationDataTopicName,
                                  MessageTopicNameGenerator defaultTopicNameGenerator)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), ros2Node, null, robotConfigurationDataTopicName,
           defaultTopicNameGenerator);
   }

   public TrackingCameraPublisher(String robotName, FullRobotModel fullRobotModel, Ros2Node ros2Node, RealtimeRos2Node realtimeRos2Node,
                                  String robotConfigurationDataTopicName, MessageTopicNameGenerator defaultTopicNameGenerator)
   {
      this.robotName = robotName;
      this.fullRobotModel = fullRobotModel;

      String generateTopicName = defaultTopicNameGenerator.generateTopicName(messageTypeToPublish);
      if (ros2Node != null)
      {
         ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         stampedPosePacketPublisher = ROS2Tools.createPublisher(ros2Node, messageTypeToPublish, generateTopicName);
         stampedPosePacketRealtimePublisher = null;
      }
      else
      {
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                              s -> robotConfigurationDataBuffer.receivedPacket(s.takeNextData()));
         stampedPosePacketPublisher = null;
         stampedPosePacketRealtimePublisher = ROS2Tools.createPublisher(realtimeRos2Node, messageTypeToPublish, generateTopicName);
      }
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

   public void receiveTrackingCameraDataFromROS(String trackingCameraDataROSTopic, URI rosCoreURI)
   {
      String graphName = robotName + "/" + name;
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, graphName, true);
      receiveTrackingCameraDataFromROS(trackingCameraDataROSTopic, rosMainNode);
   }

   public void receiveTrackingCameraDataFromROS(String trackingCameraDataROSTopic, RosMainNode rosMainNode)
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

   private final RigidBodyTransform initialTransformToWorld = new RigidBodyTransform();

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

      if(!isInitialized && sensorFrameInitializationTransformer != null)
      {         
         sensorFrameInitializationTransformer.computeTransformToWorld(fullRobotModel, initialTransformToWorld);
         isInitialized = true;
         LogTools.info("initialTransformToWorld");
         System.out.println(initialTransformToWorld);
      }
      dataToPublish.applyTransform(initialTransformToWorld);   //TODO: isInitialized should be true always.

      StampedPosePacket message = dataToPublish.toPacket();

      if (Debug)
         System.out.println("Publishing tracking camera data.");
      
      if (stampedPosePacketPublisher != null)
         stampedPosePacketPublisher.publish(message);
      else
         stampedPosePacketRealtimePublisher.publish(message);
   }

   public static interface SensorFrameInitializationTransformer
   {
      public void computeTransformToWorld(FullRobotModel fullRobotModel, RigidBodyTransform transformToWorldToPack);
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
//         transformToWorld.transform(position);
//         transformToWorld.transform(orientation);
//         transformToWorld.transform(linearVelocity);
//         transformToWorld.transform(angularVelocity);
         position.applyTransform(transformToWorld);
         orientation.applyTransform(transformToWorld);
         linearVelocity.applyTransform(transformToWorld);
         angularVelocity.applyTransform(transformToWorld);
      }

      public long getTimeStamp()
      {
         return timeStamp;
      }
      
      public StampedPosePacket toPacket()
      {
         StampedPosePacket message = new StampedPosePacket();
         
         message.getPose().setPosition(position);
         message.getPose().setOrientation(orientation);
         message.setTimestamp(timeStamp);
         message.setConfidenceFactor(confidence);
         
         return message;
      }
   }
}
