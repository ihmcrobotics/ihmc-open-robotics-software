package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
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

   public void toMessage(PrimitiveDataVectorMessage message)
   {
      StoredPropertySetMessageTools.toMessage(message, storedPropertySet);
   }

   public void fromMessage(PrimitiveDataVectorMessage message)
   {
      if (isNotFrozen())
      {
         StoredPropertySetMessageTools.fromMessage(message, storedPropertySet);
      }
   }
}
