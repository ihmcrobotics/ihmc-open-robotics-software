package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalImmutableField<T>
{
   private final RequestConfirmFreezable requestConfirmFreezable;

   private T value;

   public CRDTBidirectionalImmutableField(RequestConfirmFreezable requestConfirmFreezable, T initialValue)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;

      value = initialValue;
   }

   public T getValue()
   {
      return value;
   }

   public void setValue(T value)
   {
      if (!this.value.equals(value)) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         requestConfirmFreezable.freeze();
      }
   }

   public T toMessage()
   {
      return value;
   }

   public void fromMessage(T value)
   {
      if (!requestConfirmFreezable.isFrozen())
      {
         this.value = value;
      }
   }
}
