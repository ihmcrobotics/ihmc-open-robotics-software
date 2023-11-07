package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalBoolean
{
   private final RequestConfirmFreezable requestConfirmFreezable;

   private boolean value;

   public CRDTBidirectionalBoolean(RequestConfirmFreezable requestConfirmFreezable, boolean initialValue)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;

      value = initialValue;
   }

   public boolean getValue()
   {
      return value;
   }

   public void setValue(boolean value)
   {
      if (this.value != value) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         requestConfirmFreezable.freeze();
      }
   }

   public boolean toMessage()
   {
      return value;
   }

   public void fromMessage(boolean value)
   {
      if (!requestConfirmFreezable.isFrozen())
      {
         this.value = value;
      }
   }
}
