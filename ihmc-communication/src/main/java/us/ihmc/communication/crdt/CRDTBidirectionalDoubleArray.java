package us.ihmc.communication.crdt;

/**
 * Represents a double array that can be modified by both the
 * robot and the operator. The internal writeable instance is kept protected
 * from unchecked modifications.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalDoubleArray extends CRDTBidirectionalMutableField<double[]>
{
   public CRDTBidirectionalDoubleArray(LatestTimestampModifiable latestTimestampModifiable, int arraySize)
   {
      super(latestTimestampModifiable, new double[arraySize]);
   }

   public void getValue(double[] arrayToPack)
   {
      System.arraycopy(getValueInternal(), 0, arrayToPack, 0, arrayToPack.length);
   }

   public double getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(double[] value)
   {
      System.arraycopy(value, 0, getValueInternal(), 0, value.length);
      modify();
   }

   /** Use to prevent unecessary modifications. */
   public void setValue(int index, double value)
   {
      if (getValueReadOnly(index) != value)
         getValueAndModify()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(double[] messageArray)
   {
      for (int i = 0; i < getValueInternal().length; i++)
      {
         messageArray[i] = getValueInternal()[i];
      }
   }

   public void fromMessage(double[] messageArray)
   {
      if (isModificationIncoming())
      {
         for (int i = 0; i < getValueInternal().length; i++)
         {
            getValueInternal()[i] = messageArray[i];
         }
      }
   }
}
