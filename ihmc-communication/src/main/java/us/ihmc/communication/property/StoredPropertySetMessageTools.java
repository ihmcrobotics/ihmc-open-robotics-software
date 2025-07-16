package us.ihmc.communication.property;

import ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage;
import us.ihmc.idl.IDLSequence.Boolean;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

public class StoredPropertySetMessageTools
{
   /**
    * @param runIfChanged Will get called only if any properties are not equal to the previous
    *                     values. The parameter set will have the new values when this is called.
    */
   public static void copyToStoredPropertySet(PrimitiveDataVectorMessage message, StoredPropertySetBasics setToPack, Runnable runIfChanged)
   {
      if (!valuesAreAllEqual(message, setToPack))
      {
         fromMessage(message, setToPack);
         runIfChanged.run();
      }
   }

   public static boolean valuesAreAllEqual(PrimitiveDataVectorMessage message, StoredPropertySetReadOnly storedPropertySet)
   {
      int doubleIndex = 0;
      int integerIndex = 0;
      int booleanIndex = 0;

      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            if (storedPropertySet.get(doubleKey) != message.getDoubleValues().get(doubleIndex))
               return false;
            ++doubleIndex;
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            if (storedPropertySet.get(integerKey) != message.getIntegerValues().get(integerIndex))
               return false;
            ++integerIndex;
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            if (storedPropertySet.get(booleanKey) != (message.getBooleanValues().get(booleanIndex) == Boolean.True))
               return false;
            ++booleanIndex;
         }
      }

      return true;
   }

   public static PrimitiveDataVectorMessage newMessage(StoredPropertySetReadOnly storedPropertySet)
   {
      PrimitiveDataVectorMessage message = new PrimitiveDataVectorMessage();
      toMessage(message, storedPropertySet);
      return message;
   }

   public static void toMessage(PrimitiveDataVectorMessage message, StoredPropertySetReadOnly storedPropertySet)
   {
      message.getDoubleValues().resetQuick();
      message.getIntegerValues().resetQuick();
      message.getBooleanValues().resetQuick();

      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            message.getDoubleValues().add(storedPropertySet.get(doubleKey));
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            message.getIntegerValues().add(storedPropertySet.get(integerKey));
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            message.getBooleanValues().add(storedPropertySet.get(booleanKey));
         }
      }
   }

   public static void fromMessage(PrimitiveDataVectorMessage message, StoredPropertySetBasics storedPropertySet)
   {
      int doubleIndex = 0;
      int integerIndex = 0;
      int booleanIndex = 0;

      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            storedPropertySet.set(doubleKey, message.getDoubleValues().get(doubleIndex));
            ++doubleIndex;
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            storedPropertySet.set(integerKey, message.getIntegerValues().get(integerIndex));
            ++integerIndex;
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            storedPropertySet.set(booleanKey, message.getBooleanValues().get(booleanIndex) == Boolean.True);
            ++booleanIndex;
         }
      }
   }
}
