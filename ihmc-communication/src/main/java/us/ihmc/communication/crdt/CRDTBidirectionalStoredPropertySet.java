package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

/**
 * Represents a StoredPropertySetBasics that that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalStoredPropertySet extends CRDTBidirectionalMutableField<StoredPropertySetBasics>
{
   public CRDTBidirectionalStoredPropertySet(RequestConfirmFreezable requestConfirmFreezable, StoredPropertySetBasics storedPropertySet)
   {
      super(requestConfirmFreezable, storedPropertySet);
   }

   public StoredPropertySetReadOnly getValueReadOnly()
   {
      return getValueInternal();
   }

   /**
    * Used only for creating the RDXStoredPropertySetTuner. We should improve it.
    */
   public StoredPropertySetBasics getValueUnsafe()
   {
      return getValueInternal();
   }

   public void toMessage(PrimitiveDataVectorMessage message)
   {
      StoredPropertySetMessageTools.toMessage(message, getValueInternal());
   }

   public void fromMessage(PrimitiveDataVectorMessage message)
   {
      if (!isFrozen())
      {
         StoredPropertySetMessageTools.fromMessage(message, getValueInternal());
      }
   }
}