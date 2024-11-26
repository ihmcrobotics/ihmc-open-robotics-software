package us.ihmc.communication.crdt;

/**
 * Represents an enum that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalEnumField<T extends Enum<T>> extends CRDTBidirectionalImmutableField<T>
{
   public CRDTBidirectionalEnumField(RequestConfirmFreezable requestConfirmFreezable, T initialValue)
   {
      super(requestConfirmFreezable, initialValue);
   }

   public int toMessageOrdinal()
   {
      return toMessage() == null ? -1 : toMessage().ordinal();
   }

   /**
    * @param messageValue i.e. message.getFieldName()
    * @param enumValues T.values()
    */
   public void fromMessageOrdinal(int messageValue, T[] enumValues)
   {
      fromMessage(messageValue == -1 ? null : enumValues[messageValue]);
   }
}
