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
   private final LatestTimestampModifiable latestTimestampModifiable;

   private long value;

   public CRDTBidirectionalLong(LatestTimestampModifiable latestTimestampModifiable, long initialValue)
   {
      this.latestTimestampModifiable = latestTimestampModifiable;

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
         latestTimestampModifiable.modify();
      }
   }

   public long toMessage()
   {
      return value;
   }

   public void fromMessage(long value)
   {
      if (latestTimestampModifiable.isOutOfDate())
      {
         this.value = value;
      }
   }
}
