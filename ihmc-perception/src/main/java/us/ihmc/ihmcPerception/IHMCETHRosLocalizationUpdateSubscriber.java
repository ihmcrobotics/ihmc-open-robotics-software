package us.ihmc.ihmcPerception;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.LocalizationPointMapPacket;
import controller_msgs.msg.dds.LocalizationStatusPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import sensor_msgs.PointCloud2;
import std_msgs.Float64;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;
import us.ihmc.utilities.ros.subscriber.RosPoseStampedSubscriber;

public class IHMCETHRosLocalizationUpdateSubscriber implements Runnable, PacketConsumer<LocalizationPacket>
{
   private static final boolean DEBUG = false;
   private double overlap = 1.0;

   private final AtomicReference<UnpackedPointCloud> localizationMapPointCloud = new AtomicReference<UnpackedPointCloud>();
   private final IHMCROS2Publisher<LocalizationPointMapPacket> localizationPointMapPublisher;

   public IHMCETHRosLocalizationUpdateSubscriber(final RosMainNode rosMainNode, Ros2Node ros2Node, final PPSTimestampOffsetProvider ppsTimeOffsetProvider)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, LocalizationPacket.class, "/ihmc/localization", s -> receivedPacket(s.readNextData()));
      localizationPointMapPublisher = ROS2Tools.createPublisher(ros2Node, LocalizationPointMapPacket.class, "/ihmc/localization_point_map");

      IHMCROS2Publisher<StampedPosePacket> stampedPosePublisher = ROS2Tools.createPublisher(ros2Node, StampedPosePacket.class, "/ihmc/stamped_pose");
      RosPoseStampedSubscriber rosPoseStampedSubscriber = new RosPoseStampedSubscriber()
      {
         @Override
         protected void newPose(String frameID, TimeStampedTransform3D timeStampedTransform)
         {
            long timestamp = timeStampedTransform.getTimeStamp();
            timestamp = ppsTimeOffsetProvider.adjustTimeStampToRobotClock(timestamp);
            timeStampedTransform.setTimeStamp(timestamp);

            StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket(frameID, timeStampedTransform, overlap);
            posePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
            if (DEBUG)
               System.out.println("Pose update received. \ntimestamp: " + timeStampedTransform.getTimeStamp());

            stampedPosePublisher.publish(posePacket);
         }
      };

      rosMainNode.attachSubscriber(RosLocalizationConstants.POSE_UPDATE_TOPIC, rosPoseStampedSubscriber);

      IHMCROS2Publisher<LocalizationStatusPacket> localizationStatusPublisher = ROS2Tools.createPublisher(ros2Node, LocalizationStatusPacket.class,
                                                                                                          "/ihmc/localization_status");
      AbstractRosTopicSubscriber<Float64> overlapSubscriber = new AbstractRosTopicSubscriber<std_msgs.Float64>(std_msgs.Float64._TYPE)
      {
         @Override
         public void onNewMessage(std_msgs.Float64 message)
         {
            overlap = message.getData();
            LocalizationStatusPacket localizationOverlapPacket = HumanoidMessageTools.createLocalizationStatusPacket(overlap, null);
            localizationStatusPublisher.publish(localizationOverlapPacket);
         }
      };
      rosMainNode.attachSubscriber(RosLocalizationConstants.OVERLAP_UPDATE_TOPIC, overlapSubscriber);

      AbstractRosTopicSubscriber<std_msgs.String> statusSubscriber = new AbstractRosTopicSubscriber<std_msgs.String>(std_msgs.String._TYPE)
      {

         @Override
         public void onNewMessage(std_msgs.String message)
         {
            String status = message.getData();
            LocalizationStatusPacket localizationOverlapPacket = HumanoidMessageTools.createLocalizationStatusPacket(overlap, status);
            localizationStatusPublisher.publish(localizationOverlapPacket); // FIXME probably should not be using the same publisher as in the previous subscriber.
         }

      };

      rosMainNode.attachSubscriber(RosLocalizationConstants.STATUS_UPDATE_TOPIC, statusSubscriber);

      RosPointCloudSubscriber pointMapSubscriber = new RosPointCloudSubscriber()
      {

         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
            localizationMapPointCloud.set(pointCloudData);
         }

      };
      rosMainNode.attachSubscriber(RosLocalizationConstants.NAV_POSE_MAP, pointMapSubscriber);

      //      Thread localizationMapPublishingThread = new Thread(this);
      //      localizationMapPublishingThread.start();  

   }

   private void processAndSendPointCloud(UnpackedPointCloud pointCloudData)
   {
      Point3D[] points = pointCloudData.getPoints();
      LocalizationPointMapPacket localizationMapPacket = new LocalizationPointMapPacket();
      HumanoidMessageTools.packLocalizationPointMap(points, localizationMapPacket);
      localizationPointMapPublisher.publish(localizationMapPacket);
   }

   @Override
   public void run()
   {
      while (true)
      {
         UnpackedPointCloud pointCloud = localizationMapPointCloud.get();
         if (pointCloud != null)
         {
            processAndSendPointCloud(pointCloud);
         }
         ThreadTools.sleep(1000);
      }
   }

   @Override
   public void receivedPacket(LocalizationPacket packet)
   {
      if (packet.getReset())
      {
         localizationMapPointCloud.set(null);
      }
   }
}
