package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionInformationMessage;

import java.util.ArrayList;
import java.util.HashMap;

public class BehaviorActionReceiver<T>
{
   private final HashMap<Long, ArrayList<T>> receivedActionsByUUID = new HashMap<>();

   public void receive(T message, ActionInformationMessage actionInformationMessage, HashMap<Long, Integer> numberOfActionMessagesReceivedForUUID)
   {
      long uuid = actionInformationMessage.getSequenceUpdateUuid();

      numberOfActionMessagesReceivedForUUID.merge(uuid, 1, Integer::sum);

      ArrayList<T> list = receivedActionsByUUID.get(uuid);
      if (list == null)
      {
         ArrayList<T> arrayList = new ArrayList<>();
         arrayList.add(message);
         receivedActionsByUUID.put(uuid, arrayList);
      }
      else
      {
         list.add(message);
      }
   }

   public ArrayList<T> removeActionList(long sequenceUpdateUUID)
   {
      return receivedActionsByUUID.remove(sequenceUpdateUUID);
   }
}
