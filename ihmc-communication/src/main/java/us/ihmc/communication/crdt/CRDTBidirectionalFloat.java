package us.ihmc.communication.crdt;

public class CRDTBidirectionalFloat
{
   private final LatestTimestampModifiable latestTimestampModifiable;

   private float value;

   public CRDTBidirectionalFloat(LatestTimestampModifiable latestTimestampModifiable, float initialValue)
   {
      this.latestTimestampModifiable = latestTimestampModifiable;

      value = initialValue;
   }

   public void setInitialValue(float initialValue)
   {
      value = initialValue;
   }

   public float getValue()
   {
      return value;
   }

   public void setValue(float value)
   {
      if (this.value != value)
      {
         this.value = value;
         latestTimestampModifiable.modify();
      }
   }

   public float toMessage()
   {
      return value;
   }

   public void fromMessage(float value)
   {
      if (latestTimestampModifiable.isModificationIncoming())
      {
         this.value = value;
      }
   }
}
