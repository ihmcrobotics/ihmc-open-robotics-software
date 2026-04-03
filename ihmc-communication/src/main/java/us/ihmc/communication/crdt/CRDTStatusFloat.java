package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class CRDTStatusFloat extends CRDTStatusField
{
   private float value;

   public CRDTStatusFloat(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo, float initialValue)
   {
      super(sideThatCanModify, crdtInfo);

      value = initialValue;
   }

   public float getValue()
   {
      return value;
   }

   public void setValue(float value)
   {
      checkActorCanModifyAndMarkHasStatus();

      this.value = value;
   }

   public float toMessage()
   {
      return value;
   }

   public void fromMessage(float value)
   {
      if (isModificationDisallowed())
      {
         this.value = value;
      }
   }
}
