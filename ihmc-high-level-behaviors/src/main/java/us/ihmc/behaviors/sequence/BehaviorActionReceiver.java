package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionInformationMessage;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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

   /**
    * @return list containing all the actions received for this UUID or an empty list if no actions were received for this UUID
    */
   public List<T> removeActionList(long sequenceUpdateUUID)
   {
      ArrayList<T> removedActionList = receivedActionsByUUID.remove(sequenceUpdateUUID);
      // List.of() returns the same contant empty list on each call
      return removedActionList == null ? List.of() : removedActionList;
   }
}
