package us.ihmc.communication.crdt;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import ihmc_common_msgs.msg.dds.StoredPropertySetPrimitivesMessage;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.idl.IDLSequence.Boolean;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

public class CRDTUnidirectionalStoredPropertySet extends CRDTUnidirectionalMutableField<StoredPropertySetBasics>
{
   private final StoredPropertySetBasics storedPropertySet;

   public CRDTUnidirectionalStoredPropertySet(ROS2ActorDesignation sideThatCanModify,
                                              RequestConfirmFreezable requestConfirmFreezable,
                                              StoredPropertySetBasics storedPropertySet)
   {
      super(sideThatCanModify, requestConfirmFreezable, () -> storedPropertySet);

      this.storedPropertySet = storedPropertySet;
   }

   public StoredPropertySetReadOnly getStoredPropertySetReadOnly()
   {
      return storedPropertySet;
   }

   public void toMessage(StoredPropertySetPrimitivesMessage message)
   {
      message.getDoubleValues().clear();
      message.getIntegerValues().clear();
      message.getBooleanValues().clear();

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

   public void fromMessage(StoredPropertySetPrimitivesMessage message)
   {
      if (isNotFrozen())
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
}
