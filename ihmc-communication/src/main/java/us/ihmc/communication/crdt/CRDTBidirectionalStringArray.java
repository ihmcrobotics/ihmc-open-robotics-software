package us.ihmc.communication.crdt;

import us.ihmc.fastddsjava.cdr.idl.IDLStringSequence;

/**
 * Represents a String array that can be modified by both the
 * robot and the operator.
 * <p>
 * Warning: With this type, the data should not be continuously modified
 * tick after tick, as that will mean the value is essentially never
 * synced properly to the other side.
 */
public class CRDTBidirectionalStringArray extends CRDTBidirectionalMutableField<String[]>
{
   public CRDTBidirectionalStringArray(LatestTimestampModifiable latestTimestampModifiable, int arraySize)
   {
      super(latestTimestampModifiable, new String[arraySize]);
   }

   public String getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(int index, String value)
   {
      if (!getValueReadOnly(index).equals(value))
         getValueAndModify()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(StringBuilder[] messageArray)
   {
      for (int i = 0; i < getLength() && i < messageArray.length; ++i)
      {
         messageArray[i] = new StringBuilder(getValueReadOnly(i));
      }
   }

   public void toMessage(IDLStringSequence message)
   {
      message.clear();
      message.addAll(getValue());
   }

   public void fromMessage(StringBuilder[] messageArray)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < getLength() && i < messageArray.length; ++i)
         {
            getValueInternal()[i] = messageArray[i].toString();
         }
      }
   }

   public void fromMessage(IDLStringSequence message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < getLength() && i < message.size(); ++i)
         {
            getValueInternal()[i] = message.get(i).toString();
         }
      }
   }
}
