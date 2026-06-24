package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.jros2.ROS2Message;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class provides a simple way of creating a pool of messages that is garbage free.
 * 
 * @author Sylvain Bertrand
 */
public class MessagePool
{
   /** The list of pools, one for each message. */
   private final List<RecyclingArrayList<? extends ROS2Message<?>>> messagePoolList = new ArrayList<>();
   /** The map for retrieving a message pool given its class. */
   private final Map<Class<?>, RecyclingArrayList<? extends ROS2Message<?>>> messagePoolMap = new HashMap<>();

   /**
    * Creates a new message pool only for the given message types.
    * 
    * @param supportedMessages the list of the message types to be supported.
    */
   public MessagePool(List<Class<? extends ROS2Message<?>>> supportedMessages)
   {
      for (int i = 0; i < supportedMessages.size(); i++)
      {
         Class<? extends ROS2Message<?>> messageType = supportedMessages.get(i);
         RecyclingArrayList<? extends ROS2Message<?>> messagePool = new RecyclingArrayList<>(messageType);
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
   public <T extends ROS2Message<T>> T requestMessage(Class<T> messageClass)
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
