package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class CRDTStatusDouble extends CRDTStatusField
{
   private double value;

   public CRDTStatusDouble(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, double initialValue)
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
      checkActorCanModifyAndMarkHasStatus();

      this.value = value;
   }

   public double toMessage()
   {
      return value;
   }

   public void fromMessage(double value)
   {
      if (isModificationDisallowed())
      {
         this.value = value;
      }
   }
}
