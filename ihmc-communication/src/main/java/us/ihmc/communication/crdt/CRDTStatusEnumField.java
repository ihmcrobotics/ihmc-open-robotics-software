package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents an enum field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTStatusEnumField<T extends Enum<T>> extends CRDTStatusImmutableField<T>
{
   public CRDTStatusEnumField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, T initialValue)
   {
      super(sideThatCanModify, crdtInfo, initialValue);
   }

   public int toMessageOrdinal()
   {
      return toMessage() == null ? -1 : toMessage().ordinal();
   }

   /**
    * @param messageValue i.e. message.getFieldName()
    * @param enumValues T.values()
    */
   public void fromMessageOrdinal(int messageValue, T[] enumValues)
   {
      fromMessage(messageValue == -1 ? null : enumValues[messageValue]);
   }
}
