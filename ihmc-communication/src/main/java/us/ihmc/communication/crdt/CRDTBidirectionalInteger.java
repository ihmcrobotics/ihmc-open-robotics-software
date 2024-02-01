package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalInteger
{
   private final RequestConfirmFreezable requestConfirmFreezable;

   private int value;

   public CRDTBidirectionalInteger(RequestConfirmFreezable requestConfirmFreezable, int initialValue)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;

      value = initialValue;
   }

   public int getValue()
   {
      return value;
   }

   public void decrement()
   {
      setValue(getValue() - 1);
   }

   public void increment()
   {
      setValue(getValue() + 1);
   }

   public void setValue(int value)
   {
      if (this.value != value) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         requestConfirmFreezable.freeze();
      }
   }

   public int toMessage()
   {
      return value;
   }

   public void fromMessage(int value)
   {
      if (!requestConfirmFreezable.isFrozen())
      {
         this.value = value;
      }
   }
}
