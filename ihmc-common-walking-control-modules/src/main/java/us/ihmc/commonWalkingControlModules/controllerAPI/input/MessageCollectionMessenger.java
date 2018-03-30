package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import controller_msgs.msg.dds.MessageCollection;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;

/**
 * MessageOfMessages provides a generic way to send a collection of messages to the controller.
 */
@SuppressWarnings("rawtypes")
public class MessageCollectionMessenger
{
   private long sequenceID;

   private MessageCollection collection = new MessageCollection();
   private List<Packet<?>> packets = new ArrayList<>();
   private final Map<Class, Method> sequenceIDSetterMap = new HashMap<>();

   public MessageCollectionMessenger()
   {
      this(new Random(1651).nextLong());
   }

   public MessageCollectionMessenger(long startSequenceID)
   {
      this.sequenceID = startSequenceID;
   }

   public boolean addMessage(Packet<?> packet)
   {
      Class<? extends Packet> packetType = packet.getClass();

      Method sequenceIDSetter = sequenceIDSetterMap.get(packetType);

      if (sequenceIDSetter == null)
      {
         try
         {
            sequenceIDSetter = packetType.getMethod("setSequenceId", long.class);
            sequenceIDSetterMap.put(packetType, sequenceIDSetter);
         }
         catch (NoSuchMethodException | SecurityException e)
         {
            PrintTools.error("The message type: " + packetType.getSimpleName()
                  + " cannot be sent with a MessageCollection, it needs to declare a sequenceID field.");
            return false;
         }
      }

      try
      {
         sequenceIDSetter.invoke(packet, sequenceID);
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when setting the sequenceID for the message " + packetType.getSimpleName(), e);
      }

      packets.add(packet);
      collection.getSequences().add(sequenceID);
      sequenceID++;

      if (sequenceID == 0)
         sequenceID++;

      return true;
   }

   public void addMessages(Iterable<Packet<?>> messages)
   {
      for (Packet<?> packet : messages)
      {
         addMessage(packet);
      }
   }

   public void addMessages(Packet<?>... messages)
   {
      for (Packet<?> packet : messages)
      {
         addMessage(packet);
      }
   }

   public void sendMessageCollection(Messenger messenger)
   {
      messenger.sendMessage(collection);
      for (int i = 0; i < packets.size(); i++)
         messenger.sendMessage(packets.get(i));
   }

   public static interface Messenger
   {
      void sendMessage(Packet<?> message);
   }
}
