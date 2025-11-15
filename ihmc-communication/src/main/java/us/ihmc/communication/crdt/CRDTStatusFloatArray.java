package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a float array that should only be modified by one actor type
 * and read-only for the others. The internal writeable instance is kept protected
 * from unchecked modifications.
 */
public class CRDTStatusFloatArray extends CRDTStatusMutableField<float[]>
{
   public CRDTStatusFloatArray(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, int arraySize)
   {
      super(sideThatCanModify, crdtInfo, () -> new float[arraySize]);
   }

   public float getValueReadOnly(int index)
   {
      return getValueInternal()[index];
   }

   public void setValue(int index, float value)
   {
      accessValue()[index] = value;
   }

   public int getLength()
   {
      return getValueInternal().length;
   }

   public void toMessage(float[] messageArray)
   {
      for (int i = 0; i < getValueInternal().length; i++)
      {
         messageArray[i] = getValueInternal()[i];
      }
   }

   public void fromMessage(float[] messageArray)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         for (int i = 0; i < getValueInternal().length; i++)
         {
            getValueInternal()[i] = messageArray[i];
         }
      }
   }
}
