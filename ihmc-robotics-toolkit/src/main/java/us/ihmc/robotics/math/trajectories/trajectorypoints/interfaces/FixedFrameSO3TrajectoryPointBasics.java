package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;

public interface FixedFrameSO3TrajectoryPointBasics extends FrameSO3TrajectoryPointReadOnly, SO3TrajectoryPointBasics, FixedFrameSO3WaypointBasics
{
   default void set(double time, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(orientation, angularVelocity);
   }

   default void set(double time, FrameSO3WaypointReadOnly waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(FrameSO3TrajectoryPointReadOnly other)
   {
      set(other.getTime(), other);
   }

   default void set(ReferenceFrame referenceFrame, SO3TrajectoryPointReadOnly other)
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
   public static FixedFrameSO3TrajectoryPointBasics newFixedFrameSO3TrajectoryPointBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return newLinkedFixedFrameSO3TrajectoryPointBasics(referenceFrameHolder, new SO3TrajectoryPoint());
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
   public static FixedFrameSO3TrajectoryPointBasics newLinkedFixedFrameSO3TrajectoryPointBasics(ReferenceFrameHolder referenceFrameHolder,
                                                                                                SO3TrajectoryPointBasics originalTrajectoryPoint)
   {
      return new FixedFrameSO3TrajectoryPointBasics()
      {
         private final FixedFrameQuaternionBasics orientation = EuclidFrameFactories.newLinkedFixedFrameQuaternionBasics(referenceFrameHolder,
                                                                                                                         originalTrajectoryPoint.getOrientation());
         private final FixedFrameVector3DBasics angularVelocity = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(referenceFrameHolder,
                                                                                                                         originalTrajectoryPoint.getAngularVelocity());

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
            return EuclidHashCodeTools.toIntHashCode(getTime(), getOrientation(), getAngularVelocity());
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameSO3TrajectoryPointReadOnly)
               return equals((FrameSO3TrajectoryPointReadOnly) object);
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