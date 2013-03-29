package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import java.net.URI;
import java.util.HashMap;
import java.util.Map.Entry;

import org.ros.internal.message.Message;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

public class RosMainNode implements NodeMain
{
   private final HashMap<String, RosTopicSubscriber<? extends Message>> subscribers = new HashMap<String, RosTopicSubscriber<? extends Message>>();
   private final URI master;
   private boolean isStarted = false;
   
   
   public RosMainNode(URI master)
   {
      this.master = master;
   }
   
   public void attachSubscriber(String topicName, RosTopicSubscriber<? extends Message> subscriber)
   {
      if(isStarted)
      {
         throw new RuntimeException("Cannot attach new subscribers after execution started");
      }
      
      subscribers.put(topicName, subscriber);
   }

   @SuppressWarnings({ "unchecked", "rawtypes" })
   public void onStart(ConnectedNode connectedNode)
   {
      isStarted = true;
      
      for(Entry<String, RosTopicSubscriber<? extends Message>> entry : subscribers.entrySet())
      {
         final RosTopicSubscriber rosTopicSubscriber = entry.getValue();
         Subscriber<? extends Message> subscriber = connectedNode.newSubscriber(entry.getKey(), rosTopicSubscriber.getMessageType());
         subscriber.addMessageListener(rosTopicSubscriber);
      }
   }

   public void onShutdown(Node node)
   {
   }

   public void onShutdownComplete(Node node)
   {
   }

   public void onError(Node node, Throwable throwable)
   {
   }

   public final GraphName getDefaultNodeName()
   {
      return GraphName.of("darpaRoboticsChallenge/RosMainNode");
   }
   
   public void execute()
   {
      NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(master);
      NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(this, nodeConfiguration);
   }

}
