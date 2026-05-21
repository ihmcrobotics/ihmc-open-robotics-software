package us.ihmc.communication.crdt;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLIntSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLShortSequence;

/**
 * Represents an integer List that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalIntegerList extends CRDTBidirectionalMutableField<TIntArrayList>
{
   public CRDTBidirectionalIntegerList(LatestTimestampModifiable latestTimestampModifiable)
   {
      super(latestTimestampModifiable, new TIntArrayList());
   }

   public int getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public void setValue(int index, int value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify().set(index, value);
   }

   public void add(int value)
   {
      getValueAndModify().add(value);
   }

   public void remove(int index)
   {
      getValueAndModify().removeAt(index);
   }

   public void clear()
   {
      getValueAndModify().clear();
   }

   public int getSize()
   {
      return getValueInternal().size();
   }

   public void toMessage(int[] messageArray)
   {
      getValueInternal().toArray(messageArray, 0, Math.min(getSize(), messageArray.length));
   }

   public void toMessage(IDLIntSequence message)
   {
      message.clear();
      for (int i = 0; i < getSize(); ++i)
         message.add(getValueReadOnly(i));
   }

   public void toMessage(IDLByteSequence message)
   {
      message.getBuffer().reset();
      for (int i = 0; i < getSize(); ++i)
         message.add((byte) getValueReadOnly(i));
   }

   public void fromMessage(int[] messageArray)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         getValueInternal().add(messageArray);
      }
   }

   public void fromMessage(IDLIntSequence message)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < message.size(); ++i)
            getValueInternal().add(message.get(i));
      }
   }

   public void fromMessage(IDLByteSequence message)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < message.size(); ++i)
            getValueInternal().add(message.get(i));
      }
   }

   public void toMessage(IDLShortSequence message)
   {
      message.clear();
      for (int i = 0; i < getSize(); ++i)
         message.add((short) getValueReadOnly(i));
   }

   public void fromMessage(IDLShortSequence message)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < message.size(); ++i)
            getValueInternal().add(message.get(i));
      }
   }
}
