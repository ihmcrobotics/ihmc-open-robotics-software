package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.YoEuclideanWaypoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoEuclideanTrajectoryPoint implements EuclideanTrajectoryPointBasics
{
   private final YoEuclideanWaypoint euclideanWaypoint;
   private final YoDouble time;

   private final String namePrefix;
   private final String nameSuffix;

   public YoEuclideanTrajectoryPoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      euclideanWaypoint = new YoEuclideanWaypoint(namePrefix, nameSuffix, registry);
      time = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "time", nameSuffix), registry);
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
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
   public Point3DBasics getPosition()
   {
      return euclideanWaypoint.getPosition();
   }

   @Override
   public Vector3DBasics getLinearVelocity()
   {
      return euclideanWaypoint.getLinearVelocity();
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
}
