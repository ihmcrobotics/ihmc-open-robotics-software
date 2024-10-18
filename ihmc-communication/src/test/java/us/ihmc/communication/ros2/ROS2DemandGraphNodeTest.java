package us.ihmc.communication.ros2;

import org.junit.jupiter.api.Test;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import static org.junit.jupiter.api.Assertions.*;

public class ROS2DemandGraphNodeTest
{
   private static final double SLEEP_DURATION = 1.25 * ROS2HeartbeatMonitor.HEARTBEAT_EXPIRATION;

   @Test
   public void testIsDemanded()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "demand_graph_test_is_demanded");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ROS2Topic<Empty> testTopic = ROS2Tools.IHMC_ROOT.withSuffix("demand_graph_test_is_demanded").withType(Empty.class);

      ROS2DemandGraphNode testNode = new ROS2DemandGraphNode(ros2Helper, testTopic);
      ROS2Heartbeat testHeartbeat = new ROS2Heartbeat(ros2Node, testTopic);

      assertFalse(testNode.isDemanded());

      testHeartbeat.setAlive(true);
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertTrue(testNode.isDemanded());

      testHeartbeat.setAlive(false);
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertFalse(testNode.isDemanded());

      testHeartbeat.destroy();
      testNode.destroy();
      ros2Node.destroy();
   }

   @Test
   public void testDependantDemand()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "demand_graph_test_dependant");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ROS2Topic<Empty> testTopic = ROS2Tools.IHMC_ROOT.withSuffix("demand_graph_test_dependant").withType(Empty.class);
      ROS2Topic<Empty> dependantTopic = testTopic.withPrefix("dependant");

      ROS2Heartbeat testHeartbeat = new ROS2Heartbeat(ros2Node, testTopic);
      ROS2DemandGraphNode testNode = new ROS2DemandGraphNode(ros2Helper, testTopic);
      ROS2Heartbeat dependantHeartbeat = new ROS2Heartbeat(ros2Node, dependantTopic);
      ROS2DemandGraphNode dependantNode = new ROS2DemandGraphNode(ros2Helper, dependantTopic);
      testNode.addDependents(dependantNode);

      // No heartbeat is on -> nothing should be demanded
      assertFalse(testNode.isDemanded());
      assertFalse(dependantNode.isDemanded());

      // Dependant heartbeat is on -> both nodes should be demanded
      dependantHeartbeat.setAlive(true);
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertTrue(dependantNode.isDemanded());
      assertTrue(testNode.isDemanded());

      // Test heartbeat is on -> only test node should be demanded
      dependantHeartbeat.setAlive(false);
      testHeartbeat.setAlive(true);
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertFalse(dependantNode.isDemanded());
      assertTrue(testNode.isDemanded());

      // Neither heartbeat is on -> nothing should be demanded
      testHeartbeat.setAlive(false);
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertFalse(dependantNode.isDemanded());
      assertFalse(testNode.isDemanded());

      testNode.destroy();
      testHeartbeat.destroy();
      dependantNode.destroy();
      dependantHeartbeat.destroy();
      ros2Node.destroy();
   }

   @Test
   public void testDemandChangedCallback()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "demand_graph_test_dependant");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ROS2Topic<Empty> testTopic = ROS2Tools.IHMC_ROOT.withSuffix("demand_graph_test_dependant").withType(Empty.class);
      ROS2Topic<Empty> dependantTopic = testTopic.withPrefix("dependant");

      ROS2Heartbeat testHeartbeat = new ROS2Heartbeat(ros2Node, testTopic);
      ROS2DemandGraphNode testNode = new ROS2DemandGraphNode(ros2Helper, testTopic);
      ROS2Heartbeat dependantHeartbeat = new ROS2Heartbeat(ros2Node, dependantTopic);
      ROS2DemandGraphNode dependantNode = new ROS2DemandGraphNode(ros2Helper, dependantTopic);
      testNode.addDependents(dependantNode);

      TypedNotification<Boolean> callbackNotification = new TypedNotification<>();
      testNode.addDemandChangedCallback(callbackNotification::set);

      // Demand dependant -> callback should trigger with isDemanded = true
      dependantHeartbeat.setAlive(true);
      assertTrue(callbackNotification.blockingPoll());

      // Demand test node -> callback should NOT trigger as dependant is already demanded
      testHeartbeat.setAlive(true);
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertFalse(callbackNotification.poll());

      // Un-demand dependant -> callback should NOT trigger as test node is still demanded
      dependantHeartbeat.setAlive(false);
      ThreadTools.sleepSeconds(ROS2HeartbeatMonitor.HEARTBEAT_EXPIRATION);
      assertFalse(callbackNotification.poll());

      // Un-demand test node -> callback should trigger with isDemanded = false;
      testHeartbeat.setAlive(false);
      assertFalse(callbackNotification.blockingPoll());

      // Demand both at same time -> callback should trigger only once
      dependantHeartbeat.setAlive(true);
      testHeartbeat.setAlive(true);
      assertTrue(callbackNotification.blockingPoll());
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertFalse(callbackNotification.poll());

      // Un-demand both at same time -> callback should trigger only once
      dependantHeartbeat.setAlive(false);
      testHeartbeat.setAlive(false);
      assertFalse(callbackNotification.blockingPoll());
      ThreadTools.sleepSeconds(SLEEP_DURATION);
      assertFalse(callbackNotification.poll());

      dependantNode.destroy();
      dependantHeartbeat.destroy();
      testNode.destroy();
      testHeartbeat.destroy();
      ros2Node.destroy();
   }
}
