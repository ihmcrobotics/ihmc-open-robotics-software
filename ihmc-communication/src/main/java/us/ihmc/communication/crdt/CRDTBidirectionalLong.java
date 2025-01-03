package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalLong
{
   private final RequestConfirmFreezable requestConfirmFreezable;

   private long value;

   public CRDTBidirectionalLong(RequestConfirmFreezable requestConfirmFreezable, long initialValue)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;

      value = initialValue;
   }

   public long getValue()
   {
      return value;
   }

   public void setValue(long value)
   {
      if (this.value != value) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         requestConfirmFreezable.freeze();
      }
   }

   public long toMessage()
   {
      return value;
   }

   public void fromMessage(long value)
   {
      if (!requestConfirmFreezable.isFrozen())
      {
         this.value = value;
      }
   }
}
