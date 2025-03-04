package us.ihmc.communication.crdt;

import us.ihmc.idl.IDLSequence.Float;

public class CRDTBidirectionalFloatArray extends CRDTBidirectionalMutableField<float[]>
{
   public CRDTBidirectionalFloatArray(LatestTimestampModifiable latestTimestampModifiable, int arraySize)
   {
      super(latestTimestampModifiable, new float[arraySize]);
   }

   public float getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(int index, float value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(float[] messageArray)
   {
      System.arraycopy(getValueInternal(), 0, messageArray, 0, Math.min(getLength(), messageArray.length));
   }

   public void toMessage(Float message)
   {
      message.clear();
      message.add(getValue());
   }

   public void fromMessage(float[] messageArray)
   {
      if (isModificationIncoming())
      {
         System.arraycopy(messageArray, 0, getValueInternal(), 0, Math.min(getLength(), messageArray.length));
      }
   }

   public void fromMessage(Float message)
   {
      if (isModificationIncoming())
      {
         message.toArray(getValue());
      }
   }
}
