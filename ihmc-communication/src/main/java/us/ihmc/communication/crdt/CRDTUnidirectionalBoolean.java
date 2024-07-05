package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalBoolean extends CRDTUnidirectionalField
{
   private boolean value;

   public CRDTUnidirectionalBoolean(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, boolean initialValue)
   {
      super(sideThatCanModify, crdtInfo);

      value = initialValue;
   }

   public boolean getValue()
   {
      return value;
   }

   public void setValue(boolean value)
   {
      checkActorCanModify();

      this.value = value;
   }

   public boolean toMessage()
   {
      return value;
   }

   public void fromMessage(boolean value)
   {
      this.value = value;
   }
}
