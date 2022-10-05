package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanWaypoint;

public interface FixedFrameEuclideanWaypointBasics extends FrameEuclideanWaypointReadOnly, EuclideanWaypointBasics
{
   @Override
   FixedFramePoint3DBasics getPosition();

   @Override
   FixedFrameVector3DBasics getLinearVelocity();

   default void set(FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      getPosition().set(position);
      getLinearVelocity().set(linearVelocity);
   }

   default void set(FrameEuclideanWaypointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, EuclideanWaypointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Creates a new frame waypoint which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame waypoint.
    * @return the new linked frame waypoint.
    */
   public static FixedFrameEuclideanWaypointBasics newFixedFrameEuclideanWaypointBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameEuclideanWaypointBasics(referenceFrameHolder, new EuclideanWaypoint());
   }

   /**
    * Creates a new frame waypoint which is linked to the given frameless waypoint and
    * {@code referenceFrameHolder}.
    * <p>
    * This can essentially be used to wrap a frameless waypoint into a frame waypoint. Modifications on
    * the returned object will affect the frameless waypoint.
    * </p>
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame waypoint.
    * @param originalWaypoint     the waypoint to link to the new frame waypoint. Modifications on
    *                             either the {@code originalWaypoint} or the new frame waypoint will be
    *                             propagated to the other.
    * @return the new linked frame waypoint.
    */
   public static FixedFrameEuclideanWaypointBasics newLinkedFixedFrameEuclideanWaypointBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                              EuclideanWaypointBasics originalWaypoint)
   {
      return new FixedFrameEuclideanWaypointBasics()
      {
         private final FixedFramePoint3DBasics position = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(referenceFrameHolder,
                                                                                                                originalWaypoint.getPosition());
         private final FixedFrameVector3DBasics linearVelocity = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(referenceFrameHolder,
                                                                                                                        originalWaypoint.getLinearVelocity());

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public FixedFramePoint3DBasics getPosition()
         {
            return position;
         }

         @Override
         public FixedFrameVector3DBasics getLinearVelocity()
         {
            return linearVelocity;
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(getPosition(), getLinearVelocity());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameEuclideanWaypointReadOnly)
               return equals((FrameEuclideanWaypointReadOnly) object);
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