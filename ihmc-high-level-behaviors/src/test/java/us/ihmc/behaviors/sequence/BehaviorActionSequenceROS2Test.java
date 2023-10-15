package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceUpdateMessage;
import behavior_msgs.msg.dds.WaitDurationActionStateMessage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

public class BehaviorActionSequenceROS2Test
{
   @Test
   public void testUpdateMessage()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "action_sequence_test");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      IHMCROS2Input<ActionSequenceUpdateMessage> subscription = ros2Helper.subscribe(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC);

      ros2Helper.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, new ActionSequenceUpdateMessage());

      Assertions.assertTrue(subscription.hasReceivedFirstMessage());

      subscription.destroy();
      ros2Node.destroy();
   }

   @Test
   public void testUpdateMessageWithString()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "action_sequence_test");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      IHMCROS2Input<ActionSequenceUpdateMessage> subscription = ros2Helper.subscribe(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC);

      ActionSequenceUpdateMessage message = new ActionSequenceUpdateMessage();
      message.setSequenceSize(1);
      WaitDurationActionStateMessage waitAction = message.getWaitDurationActions().add();
      String description = "Wait for something";
      waitAction.getDefinition().getActionDefinition().getNodeDefinition().setDescription(description);
      ros2Helper.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, message);

      Assertions.assertTrue(subscription.hasReceivedFirstMessage());

      Assertions.assertEquals(description,
                              subscription.getLatest()
                                          .getWaitDurationActions()
                                          .get(0)
                                          .getDefinition()
                                          .getActionDefinition()
                                          .getNodeDefinition()
                                          .getDescriptionAsString());

      subscription.destroy();
      ros2Node.destroy();
   }
}
