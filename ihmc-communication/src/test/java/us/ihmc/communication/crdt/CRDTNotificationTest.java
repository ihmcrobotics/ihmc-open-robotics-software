package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.CRDTNotificationMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class CRDTNotificationTest
{
   @Test
   public void testCRDTNotification()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "test_crdt");
      ROS2Helper ros2 = new ROS2Helper(ros2Node);


      CRDTNotification notificationRobot = new CRDTNotification(ROS2ActorDesignation.OPERATOR, ROS2ActorDesignation.ROBOT);

      ROS2Topic<CRDTNotificationMessage> topic = ROS2Tools.IHMC_ROOT.withTypeName(CRDTNotificationMessage.class).withSuffix("crdt_topic");

      IHMCROS2Input<CRDTNotificationMessage> subscription = ros2.subscribe(topic.withIOQualifier("to_robot"));

      CRDTNotification notificationOperator0 = new CRDTNotification(ROS2ActorDesignation.OPERATOR, ROS2ActorDesignation.OPERATOR);


      CRDTNotificationMessage messageOperator0 = new CRDTNotificationMessage();
      notificationOperator0.toMessage(messageOperator0);


      ros2.publish(topic.withIOQualifier("to_robot"), messageOperator0);

      CRDTNotificationMessage receivedMessage = subscription.getMessageNotification().blockingPoll();


      // TODO: Oh actually this is gonna be kinda hard, maybe do it later


      /// Introduce 2nd operator


      CRDTNotification notificationOperator1 = new CRDTNotification(ROS2ActorDesignation.OPERATOR, ROS2ActorDesignation.OPERATOR);

      CRDTNotificationMessage messageOperator1 = new CRDTNotificationMessage();
      notificationOperator1.toMessage(messageOperator1);


      ros2Node.destroy();
   }
}
