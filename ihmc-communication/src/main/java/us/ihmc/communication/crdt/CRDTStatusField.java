package us.ihmc.communication.crdt;

import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

/**
 * A status field is not timestamped, it's one way.
 * It is for data that's continuously computed as a status
 * for an observer but not critical that it is received on
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
      checkActorCanModify();
      markHasStatus();
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

   protected void markHasStatus()
   {
      hasStatus.set();
   }

   public boolean pollHasStatus()
   {
      return hasStatus.poll();
   }
}
