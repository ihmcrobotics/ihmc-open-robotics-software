package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Represents a data field that should only be modified by one actor type
 * and read-only for the others.
 */
public class CRDTUnidirectionalDouble extends CRDTUnidirectionalField
{
   private double value;

   public CRDTUnidirectionalDouble(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable, double initialValue)
   {
      super(sideThatCanModify, requestConfirmFreezable);

      value = initialValue;
   }

   public double getValue()
   {
      return value;
   }

   public void setValue(double value)
   {
      if (this.value != value)
      {
         checkActorCanModifyAndFreeze();

         this.value = value;
      }
   }

   public double toMessage()
   {
      return value;
   }

   public void fromMessage(double value)
   {
      if (isNotFrozen())
      {
         this.value = value;
      }
   }
}
