package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLStringSequence;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a String List that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalStringList extends CRDTBidirectionalMutableField<List<String>>
{
   public CRDTBidirectionalStringList(LatestTimestampModifiable latestTimestampModifiable)
   {
      super(latestTimestampModifiable, new ArrayList<>());
   }

   public String getValueReadOnly(int index)
   {
      return getValueInternal().get(index);
   }

   public void setValue(int index, String value)
   {
      if (!getValueReadOnly(index).equals(value))
         getValueAndModify().set(index, value);
   }

   public void add(String value)
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

   public void toMessage(StringBuilder[] messageArray)
   {
      for (int i = 0; i < getSize() && i < messageArray.length; ++i)
      {
         messageArray[i] = new StringBuilder(getValueReadOnly(i));
      }
   }

   public void toMessage(IDLStringSequence message)
   {
      message.clear();
      for (int i = 0; i < getSize(); ++i)
      {
         message.add(getValueReadOnly(i));
      }
   }

   public void fromMessage(StringBuilder[] messageArray)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < messageArray.length; ++i)
         {
            getValueInternal().add(messageArray[i].toString());
         }
      }
   }

   public void fromMessage(IDLStringSequence message)
   {
      if (isModificationIncoming())
      {
         getValueInternal().clear();
         for (int i = 0; i < message.size(); ++i)
         {
            getValueInternal().add(message.get(i).toString());
         }
      }
   }
}
