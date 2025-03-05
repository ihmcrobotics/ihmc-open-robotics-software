package us.ihmc.communication.crdt;

import us.ihmc.idl.IDLSequence.StringBuilderHolder;

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

   public void toMessage(StringBuilderHolder message)
   {
      message.clear();
      for (int i = 0; i < getLength(); ++i)
      {
         message.add(getValueReadOnly(i));
      }
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

   public void fromMessage(StringBuilderHolder message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < getLength() && i < message.size(); ++i)
         {
            getValueInternal()[i] = message.getString(i);
         }
      }
   }
}
