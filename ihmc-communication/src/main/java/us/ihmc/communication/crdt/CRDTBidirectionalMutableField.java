package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 */
public class CRDTBidirectionalMutableField<T>
{
   private final LatestTimestampModifiable latestTimestampModifiable;

   private final T value;

   public CRDTBidirectionalMutableField(LatestTimestampModifiable latestTimestampModifiable, T initialValue)
   {
      this.latestTimestampModifiable = latestTimestampModifiable;

      value = initialValue;
   }

   /**
    * Used to set the initial value which can require special computation.
    */
   public T getValueToInitialize()
   {
      return value;
   }

   /**
    * Call this if you want the change to stick.
    * Do not call this every tick.
    * @return modifiable interface
    */
   public T getValueAndModify()
   {
      // Mark and timestamp modification
      modify();
      return value;
   }

   /**
    * Call this to update the data every tick, but it can get overwritten immediately by
    * incoming data. And example is to update a calculation on the robot side, but allow
    * the UI to also modify that using {@link #getValueAndModify}.
    */
   public T getValue()
   {
      return value;
   }

   /**
    * Call this when making a change to the value.
    * You may find {@link #getValueAndModify()} to be handy.
    * Do not call this every tick.
    */
   public void modify()
   {
      latestTimestampModifiable.modify(); // Mark and timestamp modification
   }

   protected T getValueInternal()
   {
      return value;
   }

   protected boolean isModificationIncoming()
   {
      return latestTimestampModifiable.isModificationIncoming();
   }
}
