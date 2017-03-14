package us.ihmc.utilities.ros;

import java.net.URI;
import java.util.LinkedHashMap;
import java.util.Map.Entry;

import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.Message;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import us.ihmc.commons.PrintTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

public class RosMainNode implements NodeMain
{
   private final LinkedHashMap<String, RosTopicSubscriberInterface<? extends Message>> subscribers = new LinkedHashMap<String, RosTopicSubscriberInterface<? extends Message>>();
   private final LinkedHashMap<String, RosTopicPublisher<? extends Message>> publishers = new LinkedHashMap<String, RosTopicPublisher<? extends Message>>();
   private final LinkedHashMap<String, RosServiceClient<? extends Message, ? extends Message>> clients = new LinkedHashMap<String, RosServiceClient<? extends Message, ? extends Message>>();
   private final LinkedHashMap<String, RosServiceServer<? extends Message, ? extends Message>> servers = new LinkedHashMap<String, RosServiceServer<? extends Message, ? extends Message>>();

   private final LinkedHashMap<String, ParameterListener> parameterListeners = new LinkedHashMap<String, ParameterListener>();
   private final LinkedHashMap<RosTopicSubscriberInterface<? extends Message>, Subscriber<? extends Message>> rosSubscribers = new LinkedHashMap<RosTopicSubscriberInterface<? extends Message>, Subscriber<? extends Message>>();

   private final URI masterURI;
   private boolean isStarted = false;

   private boolean useTf2 = false;

   private final String graphName;
   private ParameterTree parameters;

   private boolean isShutdownInProgress = false;

   private NodeMainExecutor nodeMainExecutor = null;

   private ConnectedNode connectedNode = null;

   public RosMainNode(URI masterURI, String graphName)
   {
      this(masterURI, graphName, true);
   }

   public RosMainNode(URI masterURI, String graphName, boolean useTf2)
   {
      this.masterURI = masterURI;
      this.graphName = graphName;
      this.useTf2 = useTf2;
   }

   public boolean isStarted()
   {
      return isStarted;
   }

   public ParameterTree getParameters()
   {
      return parameters;
   }

   public boolean isUseTf2()
   {
      return useTf2;
   }

   public void attachServiceClient(String topicName, RosServiceClient<? extends Message, ? extends Message> client)
   {
      checkNotStarted();

      clients.put(topicName, client);
   }

   public void attachServiceServer(String topicName, RosServiceServer<? extends Message, ? extends Message> server)
   {
      checkNotStarted();
      servers.put(topicName, server);
   }

   public void attachPublisher(String topicName, RosTopicPublisher<? extends Message> publisher)
   {
      checkNotStarted();

      publishers.put(topicName, publisher);
   }

   public void attachSubscriber(String topicName, RosTopicSubscriberInterface<? extends Message> subscriber)
   {
      checkNotStarted();

      subscribers.put(topicName, subscriber);
   }

   public void removeSubscriber(RosTopicSubscriberInterface<? extends Message> subscriber)
   {
      if (subscribers.containsValue(subscriber) && rosSubscribers.containsKey(subscriber))
      {
         Subscriber<? extends Message> rosSubscriber = rosSubscribers.get(subscriber);
         rosSubscriber.shutdown();
         rosSubscribers.remove(subscriber);
         subscribers.remove(subscriber.getMessageType());
      }
   }

   public void attachParameterListener(String topicName, ParameterListener listener)
   {
      checkNotStarted();
      parameterListeners.put(topicName, listener);
   }

   private void checkNotStarted()
   {
      if (isStarted)
      {
         throw new RuntimeException("Cannot attach new subscribers or publishers after execution started");
      }
   }

   @SuppressWarnings({ "unchecked", "rawtypes" })
   public void onStart(ConnectedNode connectedNode)
   {
      parameters = connectedNode.getParameterTree();
      for (Entry<String, RosTopicSubscriberInterface<? extends Message>> entry : subscribers.entrySet())
      {
         final RosTopicSubscriberInterface rosTopicSubscriber = entry.getValue();
         if (entry.getKey() == null)
         {
            PrintTools.error(this, "RosTopic was null! Message type: " + rosTopicSubscriber.getMessageType());
            continue;
         }
         Subscriber<? extends Message> subscriber = connectedNode.newSubscriber(entry.getKey(), rosTopicSubscriber.getMessageType());
         subscriber.addMessageListener(rosTopicSubscriber);
         rosSubscribers.put(rosTopicSubscriber, subscriber);
         rosTopicSubscriber.registered(subscriber);
         rosTopicSubscriber.connected();
      }

      for (Entry<String, RosTopicPublisher<? extends Message>> entry : publishers.entrySet())
      {
         final RosTopicPublisher rosTopicPublisher = entry.getValue();
         Publisher<? extends Message> publisher = connectedNode.newPublisher(entry.getKey(), rosTopicPublisher.getMessageType());
         rosTopicPublisher.registered(publisher);
         rosTopicPublisher.setConnectedNode(connectedNode);
         rosTopicPublisher.connected();
      }

      for (Entry<String, RosServiceServer<? extends Message, ? extends Message>> entry : servers.entrySet())
      {
         final RosServiceServer<? extends Message, ? extends Message> rosServiceServer = entry.getValue();
         ServiceServer server = connectedNode.newServiceServer(entry.getKey(), rosServiceServer.getRequestType(), rosServiceServer);
         rosServiceServer.setServiceServer(server, connectedNode, entry.getKey());
      }

      for (Entry<String, RosServiceClient<? extends Message, ? extends Message>> entry : clients.entrySet())
      {
         final RosServiceClient<? extends Message, ? extends Message> rosServiceClient = entry.getValue();

         while (!isShutdownInProgress)
         {
            try
            {
               ServiceClient client = connectedNode.newServiceClient(entry.getKey(), rosServiceClient.getRequestType());
               rosServiceClient.setServiceClient(client, connectedNode, entry.getKey());
               break;
            }
            catch (ServiceNotFoundException e)
            {
               PrintTools.error(this, "Waiting for service " + entry.getKey() + " (check spelling/service provider)...");
               ThreadTools.sleep(2000);
            }
         }
      }

      for (Entry<String, ParameterListener> entry : parameterListeners.entrySet())
      {
         connectedNode.getParameterTree().addParameterListener(entry.getKey(), entry.getValue());
      }

      this.connectedNode = connectedNode;
      isStarted = true;
   }

   public Time getCurrentTime()
   {
      if (connectedNode == null)
      {
         throw new RuntimeException("ROS Node is not connected");
      }
      return connectedNode.getCurrentTime();
   }

   public void onShutdown(Node node)
   {
      isShutdownInProgress = true;
   }

   public void onShutdownComplete(Node node)
   {
   }

   public void onError(Node node, Throwable throwable)
   {
   }

   public final GraphName getDefaultNodeName()
   {
      return GraphName.of(graphName);
   }

   public void execute()
   {
      NodeConfiguration nodeConfiguration = RosTools.createNodeConfiguration(masterURI);
      nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
      nodeMainExecutor.execute(this, nodeConfiguration);
   }

   public void shutdown()
   {
      nodeMainExecutor.shutdown();
   }
}
