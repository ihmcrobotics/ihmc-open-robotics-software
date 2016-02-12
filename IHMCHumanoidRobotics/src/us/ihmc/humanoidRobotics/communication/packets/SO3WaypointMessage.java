package us.ihmc.humanoidRobotics.communication.packets;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.trajectories.SO3WaypointInterface;

@ClassDocumentation("This class is used to build trajectory messages in taskspace. It holds the only the rotational information for one waypoint (orientation & angular velocity). "
      + "Feel free to look at EuclideanWaypoint (translational) and SE3Waypoint (rotational AND translational)")
public class SO3WaypointMessage extends IHMCRosApiPacket<SO3WaypointMessage> implements SO3WaypointInterface, TransformableDataObject<SO3WaypointMessage>
{
   @FieldDocumentation("Time at which the waypoint has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @FieldDocumentation("Define the desired 3D orientation to be reached at this waypoint. It is expressed in world frame.")
   public Quat4d orientation;
   @FieldDocumentation("Define the desired 3D angular velocity to be reached at this waypoint. It is expressed in world frame.")
   public Vector3d angularVelocity;

   public SO3WaypointMessage()
   {
   }

   public SO3WaypointMessage(SO3WaypointMessage so3Waypoint)
   {
      time = so3Waypoint.time;
      if (so3Waypoint.orientation != null)
         orientation = new Quat4d(so3Waypoint.orientation);
      if (so3Waypoint.angularVelocity != null)
         angularVelocity = new Vector3d(so3Waypoint.angularVelocity);
   }

   public SO3WaypointMessage(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      this.orientation = orientation;
      this.angularVelocity = angularVelocity;
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public Quat4d getOrientation()
   {
      return orientation;
   }

   public void setOrientation(Quat4d orientation)
   {
      this.orientation = orientation;
   }

   @Override
   public Vector3d getAngularVelocity()
   {
      return angularVelocity;
   }

   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity = angularVelocity;
   }

   @Override
   public boolean epsilonEquals(SO3WaypointMessage other, double epsilon)
   {
      if (orientation == null && other.orientation != null)
         return false;
      if (orientation != null && other.orientation == null)
         return false;

      if (angularVelocity == null && other.angularVelocity != null)
         return false;
      if (angularVelocity != null && other.angularVelocity == null)
         return false;

      if (!MathTools.epsilonEquals(time, other.time, epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;

      return true;
   }

   @Override
   public SO3WaypointMessage transform(RigidBodyTransform transform)
   {
      SO3WaypointMessage transformedWaypointMessage = new SO3WaypointMessage();

      transformedWaypointMessage.time = time;

      if (orientation != null)
         transformedWaypointMessage.orientation = TransformTools.getTransformedQuat(orientation, transform);
      else
         transformedWaypointMessage.orientation = null;

      if (angularVelocity != null)
         transformedWaypointMessage.angularVelocity = TransformTools.getTransformedVector(angularVelocity, transform);
      else
         transformedWaypointMessage.angularVelocity = null;

      return transformedWaypointMessage;
   }

   @Override
   public String toString()
   {
      return "SO3 waypoint: time = " + time + ", orientation = " + orientation + ", angular velocity = " + angularVelocity;
   }
}
