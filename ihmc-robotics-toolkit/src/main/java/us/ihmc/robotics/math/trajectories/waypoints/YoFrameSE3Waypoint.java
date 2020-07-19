package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFrameSE3Waypoint implements FrameSE3WaypointBasics
{
   private final YoFrameEuclideanWaypoint euclideanWaypoint;
   private final YoFrameSO3Waypoint so3Waypoint;

   public YoFrameSE3Waypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      euclideanWaypoint = new YoFrameEuclideanWaypoint(namePrefix, nameSuffix, registry);
      so3Waypoint = new YoFrameSO3Waypoint(namePrefix, nameSuffix, registry);
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return euclideanWaypoint.getPosition();
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocity()
   {
      return euclideanWaypoint.getLinearVelocity();
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      euclideanWaypoint.setPosition(x, y, z);
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
      so3Waypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      euclideanWaypoint.applyInverseTransform(transform);
      so3Waypoint.applyInverseTransform(transform);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      euclideanWaypoint.setReferenceFrame(referenceFrame);
      so3Waypoint.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      euclideanWaypoint.checkReferenceFrameMatch(so3Waypoint);
      return euclideanWaypoint.getReferenceFrame();
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return so3Waypoint.getOrientation();
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return so3Waypoint.getAngularVelocity();
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      so3Waypoint.setOrientation(x, y, z, s);
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      so3Waypoint.setAngularVelocity(x, y, z);
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
