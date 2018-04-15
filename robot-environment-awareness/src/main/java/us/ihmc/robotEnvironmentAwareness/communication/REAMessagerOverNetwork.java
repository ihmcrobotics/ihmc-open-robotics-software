package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;

public class REAMessagerOverNetwork implements REAMessager
{
   private static final boolean DEBUG = false;

   private final MessagerAPI messagerAPI;

   private final ConcurrentHashMap<Topic<?>, List<AtomicReference<Object>>> inputVariablesMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<REATopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<ConnectionStateListener> connectionStateListeners = new ArrayList<>();

   private final PacketCommunicator packetCommunicator;

   public static REAMessager createTCPServer(MessagerAPI messagerAPI, NetworkPorts port, NetClassList netClassList)
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(port, netClassList);
      return new REAMessagerOverNetwork(messagerAPI, packetCommunicator);
   }

   public static REAMessager createTCPClient(MessagerAPI messagerAPI, String host, NetworkPorts port, NetClassList netClassList)
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
      return new REAMessagerOverNetwork(messagerAPI, packetCommunicator);
   }

   public static REAMessager createIntraprocess(MessagerAPI messagerAPI, NetworkPorts port, NetClassList netClassList)
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port, netClassList);
      return new REAMessagerOverNetwork(messagerAPI, packetCommunicator);
   }

   private REAMessagerOverNetwork(MessagerAPI messagerAPI, PacketCommunicator packetCommunicator)
   {
      this.messagerAPI = messagerAPI;
      this.packetCommunicator = packetCommunicator;
      this.packetCommunicator.attachListener(KryoMessage.class, this::receiveREAMessage);
   }

   private <T> void receiveREAMessage(KryoMessage<T> kryoMessage)
   {
      if (kryoMessage == null)
         return;

      Message<T> message = kryoMessage.message;

      if (message == null)
         return;

      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<T> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (DEBUG)
         PrintTools.info("Packet received from network with message name: " + messageTopic.getName());

      List<AtomicReference<Object>> inputVariablesForTopic = inputVariablesMap.get(messageTopic);
      if (inputVariablesForTopic != null)
         inputVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(messageTopic);
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedMessageForTopic(message.getMessageContent()));
   }

   @Override
   public <T> void submitMessage(Message<T> message)
   {
      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<?> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (!packetCommunicator.isConnected())
      {
         PrintTools.warn(this, "This messager is closed, message's topic: " + messageTopic.getName());
         return;
      }

      if (DEBUG)
         PrintTools.info("Submit message for topic: " + messageTopic.getName());

      // Variable update over network
      packetCommunicator.send(new KryoMessage<>(message));
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);

      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(topic);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         inputVariablesMap.put(topic, boundVariablesForTopic);
      }
      boundVariablesForTopic.add((AtomicReference<Object>) boundVariable);
      return boundVariable;
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> void registerTopicListener(Topic<T> topic, REATopicListener<T> listener)
   {
      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         topicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((REATopicListener<Object>) listener);
   }

   @Override
   public void startMessager() throws IOException
   {
      packetCommunicator.connect();
   }

   @Override
   public void closeMessager()
   {
      inputVariablesMap.clear();
      packetCommunicator.closeConnection();
      packetCommunicator.disconnect();
   }

   @Override
   public boolean isMessagerOpen()
   {
      return packetCommunicator.isConnected();
   }

   @Override
   public void registerConnectionStateListener(ConnectionStateListener listener)
   {
      packetCommunicator.attachStateListener(listener);
      connectionStateListeners.add(listener);
   }

   @Override
   public void notifyConnectionStateListeners()
   {
      if (isMessagerOpen())
         connectionStateListeners.forEach(ConnectionStateListener::connected);
      else
         connectionStateListeners.forEach(ConnectionStateListener::disconnected);
   }

   @Override
   public MessagerAPI getMessagerAPI()
   {
      return messagerAPI;
   }
}
