package us.ihmc.humanoidBehaviors.waypoints;

import us.ihmc.euclid.geometry.Pose3D;

public class Waypoint
{
   /** Unique id */
   private final long uniqueId;
   /** */
   private final Pose3D pose = new Pose3D();

   /** package private */ Waypoint(long uniqueId)
   {
      this.uniqueId = uniqueId;
   }

   public Pose3D getPose()
   {
      return pose;
   }

   public long getUniqueId()
   {
      return uniqueId;
   }
}
