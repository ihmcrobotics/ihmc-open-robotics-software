package us.ihmc.robotEnvironmentAwareness.hardware;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import geometry_msgs.Quaternion;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import tf2_msgs.TFMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosTf2Publisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class MultisensetfMessageReceiver extends AbstractRosTopicSubscriber<TFMessage>
{
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "tfMessagePublisherNode");

   private final RosTf2Publisher tfMessagePublisher;

   private static final String stereoFrameID = "multisense/left_camera_frame";
   private static final String fixedFrameID = "multisense/head";

   public MultisensetfMessageReceiver() throws URISyntaxException
   {
      super(TFMessage._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "tfMessagePublisher", true);
      rosMainNode.attachSubscriber("/tf", this);

      tfMessagePublisher = new RosTf2Publisher(false);
      rosMainNode.attachPublisher("/tf", tfMessagePublisher);

      rosMainNode.execute();
   }

   @Override
   public void onNewMessage(TFMessage msg)
   {
      if (msg == null)
         return;

      long timeStempToPublish = 0;
      RigidBodyTransform transformToPublish = null;

      List<TransformStamped> transforms = msg.getTransforms();
      for (int i = 0; i < transforms.size(); i++)
      {
         boolean isHeaderMatched = transforms.get(i).getHeader().getFrameId().toString().equals(fixedFrameID);
         boolean isChildMatched = transforms.get(i).getChildFrameId().toString().equals(stereoFrameID);

         if (isHeaderMatched && isChildMatched)
         {
            TransformStamped transformStamped = transforms.get(i);
            timeStempToPublish = transformStamped.getHeader().getStamp().totalNsecs();
            Quaternion rotation = transformStamped.getTransform().getRotation();
            Vector3 translation = transformStamped.getTransform().getTranslation();

            transformToPublish = new RigidBodyTransform();

            transformToPublish.setTranslation(translation.getX(), translation.getY(), translation.getZ());
            us.ihmc.euclid.tuple4D.Quaternion quaternion = new us.ihmc.euclid.tuple4D.Quaternion(rotation.getX(), rotation.getY(), rotation.getZ(),
                                                                                                 rotation.getW());
            transformToPublish.setRotation(quaternion);
            break;
         }
      }

      if (transformToPublish == null)
         return;
      else
         tfMessagePublisher.publish(transformToPublish, timeStempToPublish, stereoFrameID, fixedFrameID);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new MultisensetfMessageReceiver();
   }
}
