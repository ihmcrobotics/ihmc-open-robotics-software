package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;
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
   public FixedFrameEuclideanWaypointBasics getEuclideanWaypoint()
   {
      return se3Waypoint.getEuclideanWaypoint();
   }

   @Override
   public FixedFrameSO3WaypointBasics getSO3Waypoint()
   {
      return se3Waypoint.getSO3Waypoint();
   }

   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      return se3Waypoint.getPosition();
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
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getTime(), getEuclideanWaypoint(), getSO3Waypoint());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameSE3TrajectoryPointReadOnly)
         return equals((FrameSE3TrajectoryPointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
