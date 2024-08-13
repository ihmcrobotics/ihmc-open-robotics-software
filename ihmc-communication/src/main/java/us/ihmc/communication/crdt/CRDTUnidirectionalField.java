package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Base class for a field that should only be modified by one actor type
 * and read-only for the others.
 */
public abstract class CRDTUnidirectionalField
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final RequestConfirmFreezable requestConfirmFreezable;
   private final CRDTInfo crdtInfo;

   public CRDTUnidirectionalField(ROS2ActorDesignation sideThatCanModify, RequestConfirmFreezable requestConfirmFreezable)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.requestConfirmFreezable = requestConfirmFreezable;

      crdtInfo = requestConfirmFreezable.getCRDTInfo();
   }

   protected void checkActorCanModifyAndFreeze()
   {
      if (isModificationDisallowed())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));

      requestConfirmFreezable.freeze();
   }

   protected boolean isModificationDisallowed()
   {
      return sideThatCanModify != crdtInfo.getActorDesignation();
   }

   protected boolean isNotFrozen()
   {
      return !requestConfirmFreezable.isFrozen();
   }
}
