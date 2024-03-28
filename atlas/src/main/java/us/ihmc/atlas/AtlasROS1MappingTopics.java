package us.ihmc.atlas;

import controller_msgs.msg.dds.RobotConfigurationData;
import geometry_msgs.PoseWithCovarianceStamped;
import org.ros.message.Time;
import sensor_msgs.PointCloud2;
import std_msgs.Header;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.publisher.RosTf2Publisher;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.types.PointType;

import java.net.URI;
import java.util.concurrent.atomic.AtomicReference;

public class AtlasROS1MappingTopics
{
   private static final String MAP_FRAME = "odom";
   private static final String ROBOT_FRAME_ID = "os_sensor";
   private static final String ROBOT_POSE_WITH_COVARIANCE_TOPIC = "robot_pose";

   private static final String OUSTER_TOPIC_IN = "/os_cloud_node/points";
   private static final String OUSTER_TOPIC_OUT = "ouster/points";

   public AtlasROS1MappingTopics()
   {
      URI rosuri = NetworkParameters.getROSURI();
      RosMainNode ros1Node = RosTools.createRosNode(rosuri, "atlas_topics");

      /* Publishers */
      RosPoseStampedPublisher poseStampedPublisher = new RosPoseStampedPublisher(false);
      RosTf2Publisher tf2Publisher = new RosTf2Publisher(false);
      RosPointCloudPublisher pointCloudPublisher = new RosPointCloudPublisher(PointType.XYZRGB, false);

      /* Subscribers */
      AtomicReference<PointCloud2> pointCloudHolder = new AtomicReference<>();
      RosPointCloudSubscriber pointCloudSubscriber = new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            pointCloudHolder.set(pointCloud);
         }
      };

      ros1Node.attachPublisher(ROBOT_POSE_WITH_COVARIANCE_TOPIC, poseStampedPublisher);
      ros1Node.attachPublisher("/tf", tf2Publisher);
      ros1Node.attachPublisher(OUSTER_TOPIC_OUT, pointCloudPublisher);

      ros1Node.attachSubscriber(OUSTER_TOPIC_IN, pointCloudSubscriber);

      AtomicReference<RobotConfigurationData> robotConfigurationDataHolder = new AtomicReference<>(new RobotConfigurationData());
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "atlas_topics2");
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic("Atlas").withTypeName(RobotConfigurationData.class),
                                  s -> robotConfigurationDataHolder.set(s.takeNextData()));

      ros1Node.execute();

      while (true)
      {
         if (ros1Node.isStarted())
         {
            RobotConfigurationData robotConfigurationData = robotConfigurationDataHolder.get();
            if (robotConfigurationData != null)
            {
               poseStampedPublisher.publish(MAP_FRAME, robotConfigurationData.getRootPosition(), robotConfigurationData.getRootOrientation());
            }

            RigidBodyTransform transform = new RigidBodyTransform();
            transform.getRotation().setToPitchOrientation(Math.toRadians(45.0));
            tf2Publisher.publish(transform, Time.fromMillis(System.currentTimeMillis()).totalNsecs(), MAP_FRAME, ROBOT_FRAME_ID);

            PointCloud2 pointCloud = pointCloudHolder.getAndSet(null);
            if (pointCloud != null)
            {
               System.out.println("publishing outster points");
//               pointCloud.getHeader().setFrameId("camera_rgb_optical_frame");
               pointCloud.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
               pointCloudPublisher.publish(pointCloud);
            }
         }

         ThreadTools.sleep(100);
      }
   }

   private static class RosPoseStampedPublisher extends RosTopicPublisher<PoseWithCovarianceStamped>
   {
      private final geometry_msgs.PoseWithCovarianceStamped initialValue;
      private int seq = 0;
      private double[] covariance = new double[36];

      public RosPoseStampedPublisher(boolean latched)
      {
         this(latched, null);

         double cov = 0.05;

         for (int i = 0; i < 6; i++)
         {
            covariance[i + 6 * i] = cov;
         }
      }

      public RosPoseStampedPublisher(boolean latched, geometry_msgs.PoseWithCovarianceStamped initialValue)
      {
         super(PoseWithCovarianceStamped._TYPE, latched);
         this.initialValue = initialValue;
      }

      @Override
      public void connected()
      {
         if(initialValue != null)
         {
            publish(initialValue);
         }
      }

      public void publish(String frameID, Tuple3DReadOnly translation, QuaternionReadOnly orientation)
      {
         PoseWithCovarianceStamped message = getMessage();

         Header header = message.getHeader();
         message.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
         header.setFrameId(frameID);
         header.setSeq(seq++);
         message.setHeader(header);
         message.getPose().setCovariance(covariance);

         message.getPose().getPose().getPosition().setX(translation.getX());
         message.getPose().getPose().getPosition().setY(translation.getY());
         message.getPose().getPose().getPosition().setZ(translation.getZ());

         message.getPose().getPose().getOrientation().setX(orientation.getX());
         message.getPose().getPose().getOrientation().setY(orientation.getY());
         message.getPose().getPose().getOrientation().setZ(orientation.getZ());
         message.getPose().getPose().getOrientation().setW(orientation.getS());

         publish(message);
      }
   }

   public static void main(String[] args)
   {
      new AtlasROS1MappingTopics();
   }
}
