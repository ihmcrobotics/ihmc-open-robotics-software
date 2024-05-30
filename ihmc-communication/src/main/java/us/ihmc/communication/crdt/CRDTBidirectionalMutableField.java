package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 */
public class CRDTBidirectionalMutableField<T>
{
   private final RequestConfirmFreezable requestConfirmFreezable;

   private final T value;

   public CRDTBidirectionalMutableField(RequestConfirmFreezable requestConfirmFreezable, T initialValue)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;

      value = initialValue;
   }

   /**
    * Call this if you want the change to stick.
    * Do not call this every tick.
    * @return modifiable interface
    */
   public T getValueAndFreeze()
   {
      requestConfirmFreezable.freeze(); // Freeze to prevent it getting overwritten immediately
      return value;
   }

   /**
    * Call this to update the data every tick, but it can get overritten immediately by
    * incoming data. And example is to update a calculation on the robot side, but allow
    * the UI to also modify that using {@link #getValueAndFreeze}.
    */
   public T getValue()
   {
      return value;
   }

   protected T getValueInternal()
   {
      return value;
   }

   protected boolean isFrozen()
   {
      return requestConfirmFreezable.isFrozen();
   }
}
