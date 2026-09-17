package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLIntSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLShortSequence;

/**
 * Represents an integer array that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalIntegerArray extends CRDTBidirectionalMutableField<int[]>
{
   public CRDTBidirectionalIntegerArray(LatestTimestampModifiable latestTimestampModifiable, int arraySize)
   {
      super(latestTimestampModifiable, new int[arraySize]);
   }

   public int getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(int index, int value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(int[] messageArray)
   {
      System.arraycopy(getValueInternal(), 0, messageArray, 0, Math.min(getLength(), messageArray.length));
   }

   public void toMessage(IDLIntSequence message)
   {
      message.clear();
      message.addAll(getValue());
   }

   public void fromMessage(int[] messageArray)
   {
      if (isModificationIncoming())
      {
         System.arraycopy(messageArray, 0, getValueInternal(), 0, Math.min(getLength(), messageArray.length));
      }
   }

   public void fromMessage(IDLIntSequence message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < Math.min(message.size(), getLength()); i++)
         {
            getValueInternal()[i] = message.get(i);
         }
      }
   }

   public void toMessage(IDLShortSequence message)
   {
      message.clear();
      int[] values = getValue();
      for (int i = 0; i < values.length; i++)
         message.add((short) values[i]);
   }

   public void fromMessage(IDLShortSequence message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < Math.min(message.size(), getLength()); i++)
            getValueInternal()[i] = Short.toUnsignedInt(message.get(i));
      }
   }
}
