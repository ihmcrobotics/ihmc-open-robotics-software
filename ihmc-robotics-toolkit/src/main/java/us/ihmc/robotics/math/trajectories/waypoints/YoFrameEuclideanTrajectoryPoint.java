package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.geometry.yoFrameObjects.YoFrameEuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanTrajectoryPointInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFrameEuclideanTrajectoryPoint implements FrameEuclideanTrajectoryPointInterface<YoFrameEuclideanTrajectoryPoint>
{
   private final YoFrameEuclideanWaypoint euclideanWaypoint;
   private final YoTrajectoryPoint trajectoryPoint;

   public YoFrameEuclideanTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      euclideanWaypoint = new YoFrameEuclideanWaypoint(namePrefix, nameSuffix, registry);
      trajectoryPoint = new YoTrajectoryPoint(namePrefix, nameSuffix, registry);
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
      trajectoryPoint.setTime(time);
   }

   @Override
   public double getTime()
   {
      return trajectoryPoint.getTime();
   }

   @Override
   public String toString()
   {
      return "Euclidean trajectory point: (time = " + WaypointToStringTools.format(getTime()) + ", " + WaypointToStringTools.waypointToString(euclideanWaypoint)
            + ")";
   }
}
