package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in taskspace. It holds the only the translational information for one trajectory point (position & linear velocity). "
      + "Feel free to look at SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational)", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class EuclideanTrajectoryPointMessage extends Packet<EuclideanTrajectoryPointMessage>
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.")
   public Point3D position = new Point3D();
   @RosExportedField(documentation = "Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3D linearVelocity = new Vector3D();

   /**
    * Empty constructor for serialization.
    */
   public EuclideanTrajectoryPointMessage()
   {
   }

   public EuclideanTrajectoryPointMessage(EuclideanTrajectoryPointMessage trajectoryPoint)
   {
      time = trajectoryPoint.time;
      if (trajectoryPoint.position != null)
         position = new Point3D(trajectoryPoint.position);
      if (trajectoryPoint.linearVelocity != null)
         linearVelocity = new Vector3D(trajectoryPoint.linearVelocity);
   }

   @Override
   public void set(EuclideanTrajectoryPointMessage other)
   {
      time = other.time;
      if (other.position != null)
         position.set(other.position);
      else
         position.set(0.0, 0.0, 0.0);
      if (other.linearVelocity != null)
         linearVelocity.set(other.linearVelocity);
      else
         linearVelocity.set(0.0, 0.0, 0.0);

      setPacketInformation(other);
   }

   public double getTime()
   {
      return time;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public void setPosition(Point3DReadOnly position)
   {
      this.position.set(position);
   }

   public Vector3DReadOnly getLinearVelocity()
   {
      return linearVelocity;
   }

   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public boolean epsilonEquals(EuclideanTrajectoryPointMessage other, double epsilon)
   {
      if (position == null ^ other.position == null)
         return false;

      if (linearVelocity == null ^ other.linearVelocity == null)
         return false;

      if (!MathTools.epsilonCompare(time, other.time, epsilon))
         return false;
      if (position != null && !position.epsilonEquals(other.position, epsilon))
         return false;
      if (linearVelocity != null && !linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String xToString = doubleFormat.format(position.getX());
      String yToString = doubleFormat.format(position.getY());
      String zToString = doubleFormat.format(position.getZ());
      String xDotToString = doubleFormat.format(linearVelocity.getX());
      String yDotToString = doubleFormat.format(linearVelocity.getY());
      String zDotToString = doubleFormat.format(linearVelocity.getZ());

      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = "position = (" + xToString + ", " + yToString + ", " + zToString + ")";
      String linearVelocityToString = "linear velocity = (" + xDotToString + ", " + yDotToString + ", " + zDotToString + ")";

      return "Euclidean trajectory point: (" + timeToString + ", " + positionToString + ", " + linearVelocityToString + ")";
   }
}
