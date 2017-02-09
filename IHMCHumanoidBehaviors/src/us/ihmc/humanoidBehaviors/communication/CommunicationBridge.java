package us.ihmc.humanoidBehaviors.communication;

import java.util.ArrayList;
import java.util.HashMap;

import javax.swing.SwingUtilities;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class CommunicationBridge implements CommunicationBridgeInterface
{
   private final PacketCommunicator packetCommunicator;
   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> listeningNetworkQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>>();
   ArrayList<CoactiveDataListenerInterface> coactiveDataListeners = new ArrayList<CoactiveDataListenerInterface>();

   private final GlobalObjectConsumer objectCosumer;

   public CommunicationBridge(PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      objectCosumer = new GlobalObjectConsumer(this);
      attachGlobalListener(objectCosumer);
      attachListener(SimpleCoactiveBehaviorDataPacket.class, new CoactiveDataListener());

   }

   public HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> getListeningNetworkQueues()
   {
      return listeningNetworkQueues;
   }

   public void consumeObjectFromNetwork(Object object)
   {
      notifyNetworkListeners(object);
   }

   private void notifyNetworkListeners(Object object)
   {
      ArrayList<ConcurrentListeningQueue> queues = listeningNetworkQueues.get(object.getClass());
      if (queues != null)
      {
         for (int i = 0; i < queues.size(); i++)
         {
            queues.get(i).put(object);
         }
      }
   }

   public void attachNetworkListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (!listeningNetworkQueues.containsKey(key))
      {
         listeningNetworkQueues.put(key, new ArrayList<ConcurrentListeningQueue>());
      }
      listeningNetworkQueues.get(key).add(queue);
   }
   
   
   public void detachNetworkListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (listeningNetworkQueues.containsKey(key))
      {
         listeningNetworkQueues.get(key).remove(queue);
      }
   }

   @Override
   public void sendPacketToController(Packet packet)
   {
      if (packetCommunicator.isConnected())
      {
         packet.setDestination(PacketDestination.CONTROLLER.ordinal());
         packetCommunicator.send(packet);
      }
   }

   @Override
   public void sendPacketToUI(Packet packet)
   {
      if (packetCommunicator.isConnected())
      {
         packet.setDestination(PacketDestination.UI.ordinal());

         packetCommunicator.send(packet);
      }
   }

   @Override
   public void sendPacketToBehavior(Packet packet)
   {
      if (packetCommunicator.isConnected())
      {
         packet.setDestination(PacketDestination.BEHAVIOR_MODULE.ordinal());
         packetCommunicator.send(packet);
      }
   }

   @Override
   public void sendPacket(Packet packet)
   {

      if (packetCommunicator.isConnected())
      {
         packetCommunicator.send(packet);
      }
   }

   @Override
   public void attachGlobalListener(GlobalPacketConsumer listener)
   {
      packetCommunicator.attachGlobalListener(listener);

   }

   @Override
   public void detachGlobalListener(GlobalPacketConsumer listener)
   {
      packetCommunicator.detachGlobalListener(listener);
   }

   @Override
   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      packetCommunicator.attachListener(clazz, listener);
   }

   @Override
   public <T extends Packet> void detachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      packetCommunicator.detachListener(clazz, listener);
   }

   public GlobalPacketConsumer getGlobalPacketConsumer()
   {
      return objectCosumer;
   }

   public void addListeners(CoactiveDataListenerInterface listener)
   {
      coactiveDataListeners.add(listener);
   }

   //TODO add the ability to easily listen for any packets

   private void notifyListeners(SimpleCoactiveBehaviorDataPacket packet)
   {
      for (int i = 0; i < coactiveDataListeners.size(); i++)
      {
         coactiveDataListeners.get(i).coactiveDataRecieved(packet);
      }
   }

   private class CoactiveDataListener implements PacketConsumer<SimpleCoactiveBehaviorDataPacket>
   {
      @Override
      public void receivedPacket(final SimpleCoactiveBehaviorDataPacket packet)
      {
         SwingUtilities.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
               notifyListeners(packet);
            }
         });
      }

   }

   public void registerYovaribleForAutoSendToUI(YoVariable var)
   {
      var.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            sendToUI(v.getName(), v.getValueAsDouble());
         }
      });
   }

   public void registerYovaribleForAutoSendToBehavior(YoVariable var)
   {
      var.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            sendToBehavior(v.getName(), v.getValueAsDouble());
         }
      });
   }

   public void sendToUI(String key, double value)
   {
      SimpleCoactiveBehaviorDataPacket newPacket = new SimpleCoactiveBehaviorDataPacket(key, value);
      sendPacketToUI(newPacket);
   }

   public void sendToUI(String key, Object data)
   {
      SimpleCoactiveBehaviorDataPacket newPacket = new SimpleCoactiveBehaviorDataPacket(key, data);
      sendPacketToUI(newPacket);
   }

   public void sendToBehavior(String key, double value)
   {
      SimpleCoactiveBehaviorDataPacket newPacket = new SimpleCoactiveBehaviorDataPacket(key, value);
      sendPacketToBehavior(newPacket);
   }

   public void sendToBehavior(String key, Object data)
   {
      SimpleCoactiveBehaviorDataPacket newPacket = new SimpleCoactiveBehaviorDataPacket(key, data);
      sendPacketToBehavior(newPacket);
   }

}
