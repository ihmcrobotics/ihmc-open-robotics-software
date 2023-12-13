package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalDouble extends CRDTUnidirectionalField
{
   private double value;

   public CRDTUnidirectionalDouble(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, double initialValue)
   {
      super(sideThatCanModify, crdtInfo);

      value = initialValue;
   }

   public double getValue()
   {
      return value;
   }

   public void setValue(double value)
   {
      checkActorCanModify();

      this.value = value;
   }

   public double toMessage()
   {
      return value;
   }

   public void fromMessage(double value)
   {
      if (isModificationDisallowed()) // Ignore updates if we are the only side that can modify
      {
         this.value = value;
      }
   }
}
