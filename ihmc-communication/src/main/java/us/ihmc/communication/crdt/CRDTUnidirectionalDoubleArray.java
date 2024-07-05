package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a double array that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTUnidirectionalDoubleArray extends CRDTUnidirectionalMutableField<double[]>
{
   public CRDTUnidirectionalDoubleArray(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, int arraySize)
   {
      super(sideThatCanModify, crdtInfo, () -> new double[arraySize]);
   }

   public double getValueReadOnly(int index)
   {
      return getValueInternal()[index];
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
      for (int i = 0; i < getValueInternal().length; i++)
      {
         getValueInternal()[i] = messageArray[i];
      }
   }
}
