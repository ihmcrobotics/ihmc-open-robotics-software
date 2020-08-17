package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFrameEuclideanTrajectoryPoint implements FrameEuclideanTrajectoryPointBasics
{
   private final YoFrameEuclideanWaypoint euclideanWaypoint;
   private final YoDouble time;

   private final String namePrefix;
   private final String nameSuffix;

   public YoFrameEuclideanTrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      euclideanWaypoint = new YoFrameEuclideanWaypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
   }

   public YoFrameEuclideanTrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      euclideanWaypoint = new YoFrameEuclideanWaypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
      setToZero(referenceFrame);
   }

   public YoDouble getYoX()
   {
      return euclideanWaypoint.getYoX();
   }

   public YoDouble getYoY()
   {
      return euclideanWaypoint.getYoY();
   }

   public YoDouble getYoZ()
   {
      return euclideanWaypoint.getYoZ();
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
      return "Euclidean trajectory point: (time = " + WaypointToStringTools.formatTime(getTime()) + ", "
            + WaypointToStringTools.waypointToString(euclideanWaypoint) + ")";
   }
}
