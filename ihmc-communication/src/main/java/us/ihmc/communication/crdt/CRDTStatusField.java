package us.ihmc.communication.crdt;

import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * A status field does not freeze (is unconfirmed).
 * It is for data that's continuously computed as a status
 * for an observer but not critical that it is recieved on
 * the other side. Typically, the robot will compute statuses
 * for monitoring and visualization purposes in the UI.
 */
public class CRDTStatusField
{
   private final ROS2ActorDesignation sideThatCanModify;
   private final CRDTInfo crdtInfo;

   /** Used to detemine whether sending this data is necessary. */
   private final Notification hasStatus = new Notification();

   public CRDTStatusField(ROS2ActorDesignation sideThatCanModify, CRDTInfo crdtInfo)
   {
      this.sideThatCanModify = sideThatCanModify;
      this.crdtInfo = crdtInfo;
   }

   protected void checkActorCanModifyAndMarkHasStatus()
   {
      if (isModificationDisallowed())
         throw new RuntimeException("%s is not allowed to modify this value.".formatted(crdtInfo.getActorDesignation()));

      hasStatus.set();
   }

   protected boolean isModificationDisallowed()
   {
      return sideThatCanModify != crdtInfo.getActorDesignation();
   }

   public boolean pollHasStatus()
   {
      return hasStatus.poll();
   }
}
