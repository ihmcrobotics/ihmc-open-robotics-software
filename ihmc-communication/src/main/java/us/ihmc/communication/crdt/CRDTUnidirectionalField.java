package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * Base class for a field that should only be modified by one actor type
 * and read-only for the others.
 */
public abstract class CRDTUnidirectionalField
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final CRDTInfo crdtInfo;

   public CRDTUnidirectionalField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.crdtInfo = crdtInfo;
   }

   protected void checkActorCanModify()
   {
      if (isModificationDisallowed())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));
   }

   protected boolean isModificationDisallowed()
   {
      return sideThatCanModify != crdtInfo.getActorDesignation();
   }
}
