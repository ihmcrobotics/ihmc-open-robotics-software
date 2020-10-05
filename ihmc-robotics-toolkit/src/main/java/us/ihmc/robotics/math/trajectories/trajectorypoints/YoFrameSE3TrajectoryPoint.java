package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFrameSE3TrajectoryPoint implements FrameSE3TrajectoryPointBasics
{
   private final YoFrameSE3Waypoint se3Waypoint;
   private final YoDouble time;

   private final String namePrefix;
   private final String nameSuffix;

   public YoFrameSE3TrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      se3Waypoint = new YoFrameSE3Waypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
   }

   public YoFrameSE3TrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      se3Waypoint = new YoFrameSE3Waypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
      setToZero(referenceFrame);
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return se3Waypoint.getPosition();
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      se3Waypoint.setPosition(x, y, z);
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocity()
   {
      return se3Waypoint.getLinearVelocity();
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      se3Waypoint.setLinearVelocity(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      se3Waypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      se3Waypoint.applyInverseTransform(transform);
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return se3Waypoint.getOrientation();
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      se3Waypoint.setOrientation(x, y, z, s);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return se3Waypoint.getAngularVelocity();
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      se3Waypoint.setAngularVelocity(x, y, z);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      se3Waypoint.setReferenceFrame(referenceFrame);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return se3Waypoint.getReferenceFrame();
   }

   @Override
   public void setTime(double time)
   {
      this.time.set(time);
   }

   @Override
   public double getTime()
   {
      return time.getValue();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   @Override
   public String toString()
   {
      return "SE3 trajectory point: (time = " + WaypointToStringTools.formatTime(getTime()) + ", " + WaypointToStringTools.waypointToString(se3Waypoint) + ")";
   }
}
