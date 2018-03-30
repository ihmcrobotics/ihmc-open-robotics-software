package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class MessagePool
{
   private final List<RecyclingArrayList<? extends Packet<?>>> messagePoolList = new ArrayList<>();
   private final Map<Class<?>, RecyclingArrayList<? extends Packet<?>>> messagePoolMap = new HashMap<>();

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

   public <T extends Packet<T>> T requestMessage(Class<T> messageClass)
   {
      @SuppressWarnings("unchecked")
      RecyclingArrayList<T> messagePool = (RecyclingArrayList<T>) messagePoolMap.get(messageClass);
      return messagePool.add();
   }

   public void reset()
   {
      for (int i = 0; i < messagePoolList.size(); i++)
         messagePoolList.get(i).clear();
   }
}
