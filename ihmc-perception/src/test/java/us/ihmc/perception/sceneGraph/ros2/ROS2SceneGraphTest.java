package us.ihmc.perception.sceneGraph.ros2;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

public class ROS2SceneGraphTest
{
   @Test
   public void testROS2SceneGraph()
   {
      SceneGraph sceneGraph = new SceneGraph();

      SceneNode child0 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child0");
      SceneNode child1 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child1");
      SceneNode child1child0 = new SceneNode(sceneGraph.getNextID().getAndIncrement(), "Child1Child0");
      sceneGraph.modifyTree(modificationQueue ->
      {
         modificationQueue.accept(new SceneGraphNodeAddition(child0, sceneGraph.getRootNode()));
         modificationQueue.accept(new SceneGraphNodeAddition(child1, sceneGraph.getRootNode()));
         modificationQueue.accept(new SceneGraphNodeAddition(child1child0, child1));
      });

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.INTRAPROCESS, "scene_graph_test");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      SceneGraph subscriptionSceneGraph = new SceneGraph();
      ROS2SceneGraphSubscription subscription = new ROS2SceneGraphSubscription(subscriptionSceneGraph, ros2Helper, ROS2IOTopicQualifier.COMMAND);
      Notification messageReceived = new Notification();
      subscription.registerMessageReceivedCallback(messageReceived::set);

      ROS2SceneGraphPublisher publisher = new ROS2SceneGraphPublisher();
      publisher.publish(sceneGraph, ros2Helper, ROS2IOTopicQualifier.COMMAND);

      messageReceived.blockingPeek();

      subscription.update();
      Assertions.assertEquals(1, subscription.getNumberOfMessagesReceived());

      Assertions.assertEquals(2, subscriptionSceneGraph.getRootNode().getChildren().size());
      SceneNode subscriptionChild0 = subscriptionSceneGraph.getRootNode().getChildren().get(0);
      Assertions.assertEquals("Child0", subscriptionChild0.getName());
      Assertions.assertEquals(0, subscriptionChild0.getChildren().size());

      SceneNode subscriptionChild1 = subscriptionSceneGraph.getRootNode().getChildren().get(1);
      Assertions.assertEquals("Child1", subscriptionChild1.getName());
      Assertions.assertEquals(1, subscriptionChild1.getChildren().size());

      SceneNode subscriptionChild1Child0 = subscriptionChild1.getChildren().get(0);
      Assertions.assertEquals("Child1Child0", subscriptionChild1Child0.getName());
      Assertions.assertEquals(0, subscriptionChild1Child0.getChildren().size());

      Assertions.assertEquals(4, subscriptionSceneGraph.getNextID().get());
      Assertions.assertEquals(4, subscriptionSceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(4, subscriptionSceneGraph.getNodeNameList().size());
      Assertions.assertEquals(4, subscriptionSceneGraph.getNamesToNodesMap().size());
      Assertions.assertEquals(4, subscriptionSceneGraph.getIDToNodeMap().size());
      Assertions.assertEquals(0, subscriptionSceneGraph.getArUcoMarkerIDToNodeMap().size());

      subscription.destroy();
      ros2Node.destroy();
   }
}
