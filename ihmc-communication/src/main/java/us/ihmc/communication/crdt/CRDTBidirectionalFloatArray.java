package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;

/**
 * Represents a float array that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalFloatArray extends CRDTBidirectionalMutableField<float[]>
{
   public CRDTBidirectionalFloatArray(LatestTimestampModifiable latestTimestampModifiable, int arraySize)
   {
      super(latestTimestampModifiable, new float[arraySize]);
   }

   public void getValue(float[] arrayToPack)
   {
      System.arraycopy(getValueInternal(), 0, arrayToPack, 0, arrayToPack.length);
   }
   
   public float getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(float[] value)
   {
      System.arraycopy(value, 0, getValueInternal(), 0, value.length);
      modify();
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

   public void toMessage(IDLFloatSequence message)
   {
      message.clear();
      for (int i = 0; i < getLength(); ++i)
      {
         message.add(getValueReadOnly(i));
      }
   }

   public void fromMessage(float[] messageArray)
   {
      if (isModificationIncoming())
      {
         System.arraycopy(messageArray, 0, getValueInternal(), 0, Math.min(getLength(), messageArray.length));
      }
   }

   public void fromMessage(IDLFloatSequence message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < message.size() && i < getLength(); ++i)
         {
            getValueInternal()[i] = message.get(i);
         }
      }
   }
}
