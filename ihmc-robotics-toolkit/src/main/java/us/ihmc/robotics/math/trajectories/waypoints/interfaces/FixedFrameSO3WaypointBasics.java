package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.SO3Waypoint;

public interface FixedFrameSO3WaypointBasics extends FrameSO3WaypointReadOnly, SO3WaypointBasics
{
   @Override
   FixedFrameQuaternionBasics getOrientation();

   @Override
   FixedFrameVector3DBasics getAngularVelocity();

   default void set(FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      getOrientation().set(orientation);
      getAngularVelocity().set(angularVelocity);
   }

   default void set(FrameSO3WaypointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceframe, SO3WaypointReadOnly other)
   {
      checkReferenceFrameMatch(referenceframe);
      set(other);
   }

   public static FixedFrameSO3WaypointBasics newFixedFrameSO3WaypointBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameSO3WaypointBasics(referenceFrameHolder, new SO3Waypoint());
   }

   public static FixedFrameSO3WaypointBasics newLinkedFixedFrameSO3WaypointBasics(ReferenceFrameHolder referenceFrameHolder, SO3WaypointBasics originalWaypoint)
   {
      return new FixedFrameSO3WaypointBasics()
      {
         private final FixedFrameQuaternionBasics orientation = EuclidFrameFactories.newLinkedFixedFrameQuaternionBasics(referenceFrameHolder,
                                                                                                                         originalWaypoint.getOrientation());
         private final FixedFrameVector3DBasics angularVelocity = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(referenceFrameHolder,
                                                                                                                         originalWaypoint.getAngularVelocity());

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public FixedFrameQuaternionBasics getOrientation()
         {
            return orientation;
         }

         @Override
         public FixedFrameVector3DBasics getAngularVelocity()
         {
            return angularVelocity;
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(getOrientation(), getAngularVelocity());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameSO3WaypointReadOnly)
               return equals((FrameSO3WaypointReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }
      };
   }
}