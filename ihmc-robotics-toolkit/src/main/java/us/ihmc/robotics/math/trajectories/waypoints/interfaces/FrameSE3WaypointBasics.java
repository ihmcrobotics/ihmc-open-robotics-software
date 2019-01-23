package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FrameSE3WaypointBasics extends FrameEuclideanWaypointBasics, FrameSO3WaypointBasics, SE3WaypointBasics
{
   default void set(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                    FrameVector3DReadOnly angularVelocity)
   {
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   default void setIncludingFrame(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                                  FrameVector3DReadOnly angularVelocity)
   {
      setReferenceFrame(position.getReferenceFrame());
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity,
                                  Vector3DReadOnly angularVelocity)
   {
      setReferenceFrame(referenceFrame);
      setPosition(position);
      setOrientation(orientation);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SE3WaypointBasics other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSE3WaypointBasics other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set(other);
   }

   default void get(FrameSE3WaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameSE3WaypointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FrameEuclideanWaypointBasics euclideanWaypointToPack, FrameSO3WaypointBasics so3WaypointToPack)
   {
      get(euclideanWaypointToPack);
      get(so3WaypointToPack);
   }

   default void getIncludingFrame(FrameEuclideanWaypointBasics euclideanWaypointToPack, FrameSO3WaypointBasics so3WaypointToPack)
   {
      getIncludingFrame(euclideanWaypointToPack);
      getIncludingFrame(so3WaypointToPack);
   }

   default void get(FixedFramePoint3DBasics positionToPack, FixedFrameQuaternionBasics orientationToPack, FixedFrameVector3DBasics linearVelocityToPack,
                    FixedFrameVector3DBasics angularVelocityToPack)
   {
      getPosition(positionToPack);
      getOrientation(orientationToPack);
      getLinearVelocity(linearVelocityToPack);
      getAngularVelocity(angularVelocityToPack);
   }

   default void getIncludingFrame(FramePoint3DBasics positionToPack, FrameQuaternionBasics orientationToPack, FrameVector3DBasics linearVelocityToPack,
                                  FrameVector3DBasics angularVelocityToPack)
   {
      getPositionIncludingFrame(positionToPack);
      getOrientationIncludingFrame(orientationToPack);
      getLinearVelocityIncludingFrame(linearVelocityToPack);
      getAngularVelocityIncludingFrame(angularVelocityToPack);
   }

   default void getPose(FixedFramePose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   default void getPoseIncludingFrame(FramePose3DBasics poseToPack)
   {
      poseToPack.setIncludingFrame(getPosition(), getOrientation());
   }

   default boolean epsilonEquals(FrameSE3WaypointBasics other, double epsilon)
   {
      boolean euclideanMatch = FrameEuclideanWaypointBasics.super.epsilonEquals(other, epsilon);
      boolean so3Match = FrameSO3WaypointBasics.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default boolean geometricallyEquals(FrameSE3WaypointBasics other, double epsilon)
   {
      boolean euclideanMatch = FrameEuclideanWaypointBasics.super.geometricallyEquals(other, epsilon);
      boolean so3Match = FrameSO3WaypointBasics.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   default void set(FrameSE3WaypointBasics other)
   {
      FrameEuclideanWaypointBasics.super.set(other);
      FrameSO3WaypointBasics.super.set(other);
   }

   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointBasics.super.setToNaN(referenceFrame);
      FrameSO3WaypointBasics.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointBasics.super.setToZero(referenceFrame);
      FrameSO3WaypointBasics.super.setToZero(referenceFrame);
   }
}
