package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public interface FixedFrameEuclideanTrajectoryPointBasics
      extends FrameEuclideanTrajectoryPointReadOnly, EuclideanTrajectoryPointBasics, FixedFrameEuclideanWaypointBasics
{
   default void set(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setTime(time);
      set(position, linearVelocity);
   }

   default void set(double time, FrameEuclideanWaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(FrameEuclideanTrajectoryPointReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, EuclideanTrajectoryPointReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Creates a new frame trajectory point which reference frame is linked to the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the reference frame holder to link to the new frame trajectory point.
    * @return the new linked frame trajectory point.
    */
   public static FixedFrameEuclideanTrajectoryPointBasics newFixedFrameEuclideanTrajectoryPointBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameEuclideanTrajectoryPointBasics(referenceFrameHolder, new EuclideanTrajectoryPoint());
   }

   /**
    * Creates a new frame trajectory point which is linked to the given frameless trajectory point and
    * {@code referenceFrameHolder}.
    * <p>
    * This can essentially be used to wrap a frameless trajectory point into a frame trajectory point.
    * Modifications on the returned object will affect the frameless trajectory point.
    * </p>
    *
    * @param referenceFrameHolder    the reference frame holder to link to the new frame trajectory
    *                                point.
    * @param originalTrajectoryPoint the trajectory point to link to the new frame trajectory point.
    *                                Modifications on either the {@code originalTrajectoryPoint} or the
    *                                new frame trajectory point will be propagated to the other.
    * @return the new linked frame trajectory point.
    */
   public static FixedFrameEuclideanTrajectoryPointBasics newLinkedFixedFrameEuclideanTrajectoryPointBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                                            EuclideanTrajectoryPointBasics originalTrajectoryPoint)
   {
      return new FixedFrameEuclideanTrajectoryPointBasics()
      {
         private final FixedFramePoint3DBasics position = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(referenceFrameHolder,
                                                                                                                originalTrajectoryPoint.getPosition());
         private final FixedFrameVector3DBasics linearVelocity = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(referenceFrameHolder,
                                                                                                                        originalTrajectoryPoint.getLinearVelocity());

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
         public void setTime(double time)
         {
            originalTrajectoryPoint.setTime(time);
         }

         @Override
         public double getTime()
         {
            return originalTrajectoryPoint.getTime();
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(getTime(), getPosition(), getLinearVelocity());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameEuclideanTrajectoryPointReadOnly)
               return equals((FrameEuclideanTrajectoryPointReadOnly) object);
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