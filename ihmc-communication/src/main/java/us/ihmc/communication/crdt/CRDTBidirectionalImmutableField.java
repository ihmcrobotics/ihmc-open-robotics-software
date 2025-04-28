package us.ihmc.communication.crdt;

import java.util.Objects;

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
   private final LatestTimestampModifiable latestTimestampModifiable;

   private T value;

   public CRDTBidirectionalImmutableField(LatestTimestampModifiable latestTimestampModifiable, T initialValue)
   {
      this.latestTimestampModifiable = latestTimestampModifiable;

      value = initialValue;
   }

   public T getValue()
   {
      return value;
   }

   public void setValue(T value)
   {
      if (!Objects.equals(this.value, value)) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         latestTimestampModifiable.modify();
      }
   }

   public T toMessage()
   {
      return value;
   }

   public void fromMessage(T value)
   {
      if (latestTimestampModifiable.isModificationIncoming())
      {
         this.value = value;
      }
   }
}
