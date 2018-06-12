package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.packets.Packet;
import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * This class provides a simple way of creating a pool of messages that is garbage free.
 * 
 * @author Sylvain Bertrand
 */
public class MessagePool
{
   /** The list of pools, one for each message. */
   private final List<RecyclingArrayList<? extends Packet<?>>> messagePoolList = new ArrayList<>();
   /** The map for retrieving a message pool given its class. */
   private final Map<Class<?>, RecyclingArrayList<? extends Packet<?>>> messagePoolMap = new HashMap<>();

   /**
    * Creates a new message pool only for the given message types.
    * 
    * @param supportedMessages the list of the message types to be supported.
    */
   public MessagePool(List<Class<? extends Packet<?>>> supportedMessages)
   {
      for (int i = 0; i < supportedMessages.size(); i++)
      {
         Class<? extends Packet<?>> messageType = supportedMessages.get(i);
         RecyclingArrayList<? extends Packet<?>> messagePool = new RecyclingArrayList<>(messageType);
         messagePoolMap.put(messageType, messagePool);
         messagePoolList.add(messagePool);
      }
   }

   /**
    * Requests an available instance of the given type.
    * 
    * @param messageClass the type to get an instance of.
    * @return the instance.
    */
   public <T extends Packet<T>> T requestMessage(Class<T> messageClass)
   {
      @SuppressWarnings("unchecked")
      RecyclingArrayList<T> messagePool = (RecyclingArrayList<T>) messagePoolMap.get(messageClass);
      return messagePool.add();
   }

   /**
    * Marks all the internal memory as available for use.
    * <p>
    * Once this method has been called, messages previously requested will be recycled.
    * </p>
    */
   public void reset()
   {
      for (int i = 0; i < messagePoolList.size(); i++)
         messagePoolList.get(i).clear();
   }
}
