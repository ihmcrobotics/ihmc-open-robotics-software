package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents an enum List that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalEnumList<T extends Enum<T>> extends CRDTBidirectionalMutableField<List<T>>
{
   public CRDTBidirectionalEnumList(LatestTimestampModifiable latestTimestampModifiable)
   {
      super(latestTimestampModifiable, new ArrayList<>());
   }

   public T getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public void setValue(int index, T value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify().set(index, value);
   }

   public void add(T value)
   {
      getValueAndModify().add(value);
   }

   public void remove(int index)
   {
      getValueAndModify().remove(index);
   }

   public void clear()
   {
      getValueAndModify().clear();
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public void toMessage(IDLByteSequence messageArray)
   {
      List<T> values = getValueInternal();
      messageArray.clear();
      for (int i = 0; i < getSize(); i++)
      {
         messageArray.add(values.get(i) == null ? -1 : (byte) values.get(i).ordinal());
      }
   }

   public void toMessage(byte[] messageArray)
   {
      List<T> values = getValueInternal();
      for (int i = 0; i < Math.min(getSize(), messageArray.length); i++)
      {
         messageArray[i] = values.get(i) == null ? -1 : (byte) values.get(i).ordinal();
      }
   }

   public void fromMessage(IDLByteSequence messageArray, T[] enumValues)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < messageArray.size(); i++)
         {
            getValueInternal().add(messageArray.get(i) == -1 ? null : enumValues[messageArray.get(i)]);
         }
      }
   }

   public void fromMessage(byte[] messageArray, T[] enumValues)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < messageArray.length; i++)
         {
            getValueInternal().add(messageArray[i] == -1 ? null : enumValues[messageArray[i]]);
         }
      }
   }
}
