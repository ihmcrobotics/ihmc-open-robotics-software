package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class CRDTInfo
{
   private final ROS2ActorDesignation actorDesignation;
   private final int maxFreezeDuration;
   private long updateNumber = 0; // Increment before first update

   public CRDTInfo(ROS2ActorDesignation actorDesignation, int maxFreezeDuration)
   {
      this.actorDesignation = actorDesignation;
      this.maxFreezeDuration = maxFreezeDuration;
   }

   public void startNextUpdate()
   {
      ++updateNumber;
   }

   public long getUpdateNumber()
   {
      return updateNumber;
   }

   public int getMaxFreezeDuration()
   {
      return maxFreezeDuration;
   }

   public ROS2ActorDesignation getActorDesignation()
   {
      return actorDesignation;
   }
}
