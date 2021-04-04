package us.ihmc.humanoidBehaviors.waypoints;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;

public class Waypoint
{
   /** Unique id */
   private long uniqueId = -1;
   /** */
   private final Pose3D pose = new Pose3D();

   private Waypoint()
   {
      // for kryo only
   }

   /** package private */ Waypoint(long uniqueId)
   {
      this.uniqueId = uniqueId;
   }

   public Pose3DBasics getPose()
   {
      return pose;
   }

   public long getUniqueId()
   {
      return uniqueId;
   }
}
