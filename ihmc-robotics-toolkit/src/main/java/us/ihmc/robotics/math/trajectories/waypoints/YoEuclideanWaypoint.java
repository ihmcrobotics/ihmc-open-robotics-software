package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoEuclideanWaypoint implements EuclideanWaypointBasics
{
   private final YoPoint3D position;
   private final YoVector3D linearVelocity;

   public YoEuclideanWaypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      position = new YoPoint3D(namePrefix + "Position", nameSuffix, registry);
      linearVelocity = new YoVector3D(namePrefix + "LinearVelocity", nameSuffix, registry);
   }

   public YoDouble getYoX()
   {
      return position.getYoX();
   }

   public YoDouble getYoY()
   {
      return position.getYoY();
   }

   public YoDouble getYoZ()
   {
      return position.getYoZ();
   }

   @Override
   public Point3DBasics getPosition()
   {
      return position;
   }

   @Override
   public Vector3DBasics getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getLinearVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof EuclideanWaypointReadOnly)
         return equals((EuclideanWaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
