package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;

/**
 * Represents an enum array that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalEnumArray<T extends Enum<T>> extends CRDTBidirectionalMutableField<T[]>
{
   public CRDTBidirectionalEnumArray(LatestTimestampModifiable latestTimestampModifiable, T[] initialArray)
   {
      super(latestTimestampModifiable, initialArray);
   }

   public T getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(int index, T value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(IDLByteSequence messageArray)
   {
      T[] values = getValueInternal();
      messageArray.getBuffer().reset();
      for (int i = 0; i < getLength(); i++)
      {
         messageArray.add(values[i] == null ? -1 : (byte) values[i].ordinal());
      }
   }

   public void toMessage(byte[] messageArray)
   {
      T[] values = getValueInternal();
      for (int i = 0; i < Math.min(getLength(), messageArray.length); i++)
      {
         messageArray[i] = values[i] == null ? -1 : (byte) values[i].ordinal();
      }
   }

   public void fromMessage(IDLByteSequence messageArray, T[] enumValues)
   {
      if (isModificationIncoming())
      {
         T[] values = getValueInternal();
         for (int i = 0; i < Math.min(getLength(), messageArray.size()); i++)
         {
            values[i] = messageArray.get(i) == -1 ? null : enumValues[messageArray.get(i)];
         }
      }
   }

   public void fromMessage(byte[] messageArray, T[] enumValues)
   {
      if (isModificationIncoming())
      {
         T[] values = getValueInternal();
         for (int i = 0; i < Math.min(getLength(), messageArray.length); i++)
         {
            values[i] = messageArray[i] == -1 ? null : enumValues[messageArray[i]];
         }
      }
   }
}
