package us.ihmc.communication.crdt;

import us.ihmc.idl.IDLSequence.Boolean;

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

   public void toMessage(Boolean message)
   {
      message.clear();
      for (int i = 0; i < getLength(); ++i)
      {
         message.add(getValueReadOnly(i));
      }
   }

   public void fromMessage(boolean[] messageArray)
   {
      if (isModificationIncoming())
      {
         System.arraycopy(messageArray, 0, getValueInternal(), 0, Math.min(getLength(), messageArray.length));
      }
   }

   public void fromMessage(Boolean message)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < message.size() && i < getLength(); ++i)
         {
            setValue(i, message.getBoolean(i));
         }
      }
   }
}
