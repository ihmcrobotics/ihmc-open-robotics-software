package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;

public class FrameEuclideanTrajectoryPoint implements FrameEuclideanTrajectoryPointBasics
{
   private final FrameEuclideanWaypoint euclideanWaypoint = new FrameEuclideanWaypoint();
   private double time;

   public FrameEuclideanTrajectoryPoint()
   {
   }

   public FrameEuclideanTrajectoryPoint(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameEuclideanTrajectoryPoint(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      setIncludingFrame(time, position, linearVelocity);
   }

   public FrameEuclideanTrajectoryPoint(FrameEuclideanTrajectoryPointBasics other)
   {
      setIncludingFrame(other);
   }

   public FrameEuclideanTrajectoryPoint(ReferenceFrame referenceFrame, EuclideanTrajectoryPointBasics other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return euclideanWaypoint.getPosition();
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      euclideanWaypoint.setPosition(x, y, z);
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocity()
   {
      return euclideanWaypoint.getLinearVelocity();
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      euclideanWaypoint.setLinearVelocity(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      euclideanWaypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      euclideanWaypoint.applyInverseTransform(transform);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      euclideanWaypoint.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return euclideanWaypoint.getReferenceFrame();
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public String toString()
   {
      return "Euclidean trajectory point: (time = " + WaypointToStringTools.format(getTime()) + ", " + WaypointToStringTools.waypointToString(euclideanWaypoint)
            + ")";
   }
}
