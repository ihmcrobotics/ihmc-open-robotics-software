package us.ihmc.communication.property;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

import java.util.Arrays;
import java.util.List;

public class StoredPropertySetMessageTools
{
   public static void copyToStoredPropertySet(StoredPropertySetMessage message, StoredPropertySetBasics setToPack, Runnable runIfChanged)
   {
      List<String> values = Arrays.asList(message.getStrings().toStringArray());

      if (!setToPack.getAllAsStrings().equals(values))
      {
         setToPack.setAllFromStrings(values);
         runIfChanged.run();
      }
   }

   public static boolean valuesAreAllEqual(StoredPropertySetMessage message, StoredPropertySetReadOnly set)
   {
      List<String> values = Arrays.asList(message.getStrings().toStringArray());
      return set.getAllAsStrings().equals(values);
   }

   public static StoredPropertySetMessage newMessage(StoredPropertySetBasics storedPropertySet)
   {
      StoredPropertySetMessage storedPropertySetMessage = new StoredPropertySetMessage();
      storedPropertySet.getAllAsStrings().forEach(value -> storedPropertySetMessage.getStrings().add(value));
      return storedPropertySetMessage;
   }
}
