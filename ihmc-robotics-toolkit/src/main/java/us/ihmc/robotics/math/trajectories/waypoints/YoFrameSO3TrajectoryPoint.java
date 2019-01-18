package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.geometry.yoFrameObjects.YoFrameSO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3TrajectoryPointInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFrameSO3TrajectoryPoint implements FrameSO3TrajectoryPointInterface
{
   private final YoFrameSO3Waypoint so3Waypoint;
   private final YoTrajectoryPoint trajectoryPoint;

   public YoFrameSO3TrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      so3Waypoint = new YoFrameSO3Waypoint(namePrefix, nameSuffix, registry);
      trajectoryPoint = new YoTrajectoryPoint(namePrefix, nameSuffix, registry);
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      so3Waypoint.setOrientation(x, y, z, s);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      so3Waypoint.setAngularVelocity(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      so3Waypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      so3Waypoint.applyInverseTransform(transform);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      so3Waypoint.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return so3Waypoint.getReferenceFrame();
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
      return "SO3 trajectory point: (time = " + WaypointToStringTools.format(getTime()) + ", " + WaypointToStringTools.waypointToString(so3Waypoint) + ")";
   }
}
