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
   private final Confirmable confirmable;

   private int value;

   public CRDTBidirectionalInteger(Confirmable confirmable, int initialValue)
   {
      this.confirmable = confirmable;

      value = initialValue;
   }

   public int intValue()
   {
      return value;
   }

   public void decrement()
   {
      setValue(intValue() - 1);
   }

   public void increment()
   {
      setValue(intValue() + 1);
   }

   public void setValue(int value)
   {
      if (this.value != value) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         confirmable.freeze();
      }
   }

   public void fromMessage(int value)
   {
      if (!confirmable.isFrozen())
      {
         this.value = value;
      }
   }

   public int toMessage()
   {
      return value;
   }
}
