package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTStatusLong extends CRDTStatusField
{
   private long value;

   public CRDTStatusLong(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, long initialValue)
   {
      super(sideThatCanModify, crdtInfo);

      value = initialValue;
   }

   public long getValue()
   {
      return value;
   }

   public void setValue(long value)
   {
      checkActorCanModifyAndMarkHasStatus();

      this.value = value;
   }

   public long toMessage()
   {
      return value;
   }

   public void fromMessage(long value)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         this.value = value;
      }
   }
}
