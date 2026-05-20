package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLBoolSequence;

/**
 * Represents a boolean array that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalBooleanArray extends CRDTBidirectionalMutableField<boolean[]>
{
   public CRDTBidirectionalBooleanArray(LatestTimestampModifiable latestTimestampModifiable, int arraySize)
   {
      super(latestTimestampModifiable, new boolean[arraySize]);
   }

   public boolean getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(int index, boolean value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(boolean[] messageArray)
   {
      System.arraycopy(getValueInternal(), 0, messageArray, 0, Math.min(getLength(), messageArray.length));
   }

   public void toMessage(IDLBoolSequence message)
   {
      message.clear();
      message.addAll(getValue());
   }

   public void fromMessage(boolean[] messageArray)
   {
      if (isModificationIncoming())
      {
         System.arraycopy(messageArray, 0, getValueInternal(), 0, Math.min(getLength(), messageArray.length));
      }
   }

   public void fromMessage(IDLBoolSequence message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < message.size() && i < getLength(); ++i)
         {
            setValue(i, message.get(i));
         }
      }
   }
}
