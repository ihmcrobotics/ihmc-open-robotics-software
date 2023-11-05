package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalField<T>
{
   private final Confirmable confirmable;
   private final CRDTInfo crdtInfo;

   private T value;

   public CRDTBidirectionalField(Confirmable confirmable, CRDTInfo crdtInfo, T initialValue)
   {
      this.confirmable = confirmable;
      this.crdtInfo = crdtInfo;

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
         confirmable.freeze();
      }
   }

   public void fromMessage(T value)
   {
      if (!confirmable.isFrozen())
      {
         this.value = value;
      }
   }

   public T toMessage()
   {
      return value;
   }
}
