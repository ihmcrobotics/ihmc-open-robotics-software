package us.ihmc.humanoidRobotics.communication.packets;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.trajectories.EuclideanWaypointInterface;

@ClassDocumentation("This class is used to build trajectory messages in taskspace. It holds the only the translational information for one waypoint (position & linear velocity). "
      + "Feel free to look at SO3Waypoint (rotational) and SE3Waypoint (rotational AND translational)")
public class EuclideanWaypointMessage extends IHMCRosApiPacket<EuclideanWaypointMessage> implements EuclideanWaypointInterface, TransformableDataObject<EuclideanWaypointMessage>
{
   @FieldDocumentation("Time at which the waypoint has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @FieldDocumentation("Define the desired 3D position to be reached at this waypoint. It is expressed in world frame.")
   public Point3d position;
   @FieldDocumentation("Define the desired 3D linear velocity to be reached at this waypoint. It is expressed in world frame.")
   public Vector3d linearVelocity;

   public EuclideanWaypointMessage()
   {
   }

   public EuclideanWaypointMessage(EuclideanWaypointMessage euclideanWaypointMessage)
   {
      if (euclideanWaypointMessage.position != null)
         position = new Point3d(euclideanWaypointMessage.position);
      if (euclideanWaypointMessage.linearVelocity != null)
         linearVelocity = new Vector3d(euclideanWaypointMessage.linearVelocity);
      time = euclideanWaypointMessage.time;
   }

   public EuclideanWaypointMessage(double time, Point3d position, Vector3d linearVelocity)
   {
      this.position = position;
      this.linearVelocity = linearVelocity;
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
   public Point3d getPosition()
   {
      return position;
   }

   public void setPosition(Point3d position)
   {
      this.position = position;
   }

   @Override
   public Vector3d getLinearVelocity()
   {
      return linearVelocity;
   }

   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity = linearVelocity;
   }

   @Override
   public boolean epsilonEquals(EuclideanWaypointMessage other, double epsilon)
   {
      if (position == null && other.position != null)
         return false;
      if (position != null && other.position == null)
         return false;

      if (linearVelocity == null && other.linearVelocity != null)
         return false;
      if (linearVelocity != null && other.linearVelocity == null)
         return false;

      if (!MathTools.epsilonEquals(time, other.time, epsilon))
         return false;
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;

      return true;
   }

   @Override
   public EuclideanWaypointMessage transform(RigidBodyTransform transform)
   {
      EuclideanWaypointMessage transformedWaypointMessage = new EuclideanWaypointMessage();

      transformedWaypointMessage.time = time;

      if (position != null)
         transformedWaypointMessage.position = TransformTools.getTransformedPoint(position, transform);
      else
         transformedWaypointMessage.position = null;

      if (linearVelocity != null)
         transformedWaypointMessage.linearVelocity = TransformTools.getTransformedVector(linearVelocity, transform);
      else
         transformedWaypointMessage.linearVelocity = null;

      return transformedWaypointMessage;
   }
}
