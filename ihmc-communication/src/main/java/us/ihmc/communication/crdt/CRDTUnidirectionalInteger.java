package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalInteger
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final CRDTInfo crdtInfo;

   private int value;

   public CRDTUnidirectionalInteger(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, int initialValue)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.crdtInfo = crdtInfo;

      value = initialValue;
   }

   public int getValue()
   {
      return value;
   }

   public void setValue(int value)
   {
      if (sideThatCanModify != crdtInfo.getActorDesignation())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));

      this.value = value;
   }

   public int toMessage()
   {
      return value;
   }

   public void fromMessage(int value)
   {
      if (sideThatCanModify != crdtInfo.getActorDesignation()) // Ignore updates if we are the only side that can modify
      {
         this.value = value;
      }
   }
}
