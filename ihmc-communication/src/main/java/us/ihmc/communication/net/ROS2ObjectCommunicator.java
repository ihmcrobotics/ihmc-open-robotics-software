package us.ihmc.communication.net;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.interfaces.IHMCInterfaces;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

public class ROS2ObjectCommunicator implements NetworkedObjectCommunicator
{
   /** FAST_RTPS or INTRAPROCESS */
   private final PubSubImplementation pubSubImplementation;
   /** Topic name for communication */
   private final String topicName;
   /** ROS2 node for publishing and subscribing */
   private final RealtimeRos2Node realtimeRos2Node;
   /** Class map */
   private final HashMap<Class<?>, ROS2CommunicatorClass<?>> ros2CommunicatorClasses = new HashMap<>();
   /** Global consumers for ROS2 */
   private final List<GlobalObjectConsumer> globalConsumers = new ArrayList<>();

   public ROS2ObjectCommunicator(PubSubImplementation pubSubImplementation, NetworkPorts topic, NetClassList netClassList)
   {
      this.pubSubImplementation = pubSubImplementation;
      topicName = topic.getName();
      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, topicName, ROS2Tools.RUNTIME_EXCEPTION);
      for (Class<?> messageClass : netClassList.getPacketClassList())
      {
         if (IHMCInterfaces.contains(messageClass))
         {
            if (messageClass.getSimpleName().contains("Robot"))
            PrintTools.info("Registering class:" + messageClass.getSimpleName());
            ros2CommunicatorClasses.put(messageClass,
                                        new ROS2CommunicatorClass<>(pubSubImplementation, realtimeRos2Node, messageClass, topicName, globalConsumers));
         }
      }
      
//      attachGlobalListener((object) -> {
//         PrintTools.info(this, "Received: " + object);
//      });
   }

   @Override
   public int send(Object object)
   {
      if (ros2CommunicatorClasses.containsKey(object.getClass()))
      {
         int i = ros2CommunicatorClasses.get(object.getClass()).getPublisher().publish(object) ? 1 : 0;

         return i;
      }
      else
         return 1;
   }

   @Override
   public void attachStateListener(TcpNetStateListener stateListener)
   {

   }

   @Override
   public void closeConnection()
   {

   }

   @Override
   public void attachStateListener(ConnectionStateListener stateListener)
   {

   }

   @Override
   public <T> void attachListener(Class<T> clazz, ObjectConsumer<T> listener)
   {
      if (ros2CommunicatorClasses.containsKey(clazz))
         ros2CommunicatorClasses.get(clazz).addSubscriber(listener);
   }

   @Override
   public <T> void detachListener(Class<T> clazz, ObjectConsumer<T> listener)
   {

   }

   @Override
   public void attachGlobalListener(GlobalObjectConsumer listener)
   {
      globalConsumers.add(listener);
   }

   @Override
   public void detachGlobalListener(GlobalObjectConsumer listener)
   {

   }

   @Override
   public void disconnect() throws IOException
   {

   }

   @Override
   public void connect() throws IOException
   {
      realtimeRos2Node.spin();
   }

   @Override
   public boolean isConnected()
   {
      return true;
   }

   @Override
   public void consumeObject(Object object)
   {

   }
}
