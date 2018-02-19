package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in taskspace. It holds the only the translational information for one trajectory point (position & linear velocity). "
      + "Feel free to look at SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational)", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class EuclideanTrajectoryPointMessage extends Packet<EuclideanTrajectoryPointMessage>
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.")
   public Point3D position;
   @RosExportedField(documentation = "Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3D linearVelocity;

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

   public void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   public void setPosition(Point3DReadOnly position)
   {
      if (this.position == null)
         this.position = new Point3D(position);
      else
         this.position.set(position);
   }

   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      if (this.linearVelocity == null)
         this.linearVelocity = new Vector3D(linearVelocity);
      else
         this.linearVelocity.set(linearVelocity);
   }

   public void setTimeToZero()
   {
      time = 0.0;
   }

   public void setPositionToZero()
   {
      position.set(0.0, 0.0, 0.0);
   }

   public void setLinearVelocityToZero()
   {
      linearVelocity.set(0.0, 0.0, 0.0);
   }

   public void setToZero()
   {
      setTimeToZero();
      setPositionToZero();
      setLinearVelocityToZero();
   }

   public void setTimeToNaN()
   {
      time = Double.NaN;
   }

   public void setPositionToNaN()
   {
      position.set(Double.NaN, Double.NaN, Double.NaN);
   }

   public void setLinearVelocityToNaN()
   {
      linearVelocity.set(Double.NaN, Double.NaN, Double.NaN);
   }

   public void setToNaN()
   {
      setTimeToNaN();
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   public double positionDistance(EuclideanTrajectoryPointMessage other)
   {
      return position.distance(other.position);
   }

   public double getX()
   {
      return position.getX();
   }

   public double getY()
   {
      return position.getY();
   }

   public double getZ()
   {
      return position.getZ();
   }

   public boolean containsNaN()
   {
      if (Double.isNaN(time))
         return true;
      if (Double.isNaN(position.getX()) || Double.isNaN(position.getY()) || Double.isNaN(position.getZ()))
         return true;
      if (Double.isNaN(linearVelocity.getX()) || Double.isNaN(linearVelocity.getY()) || Double.isNaN(linearVelocity.getZ()))
         return true;
      return false;
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

   public boolean geometricallyEquals(EuclideanTrajectoryPointMessage other, double epsilon)
   {
      if (position == null ^ other.position == null)
         return false;

      if (linearVelocity == null ^ other.linearVelocity == null)
         return false;

      if (!MathTools.epsilonCompare(time, other.time, epsilon))
         return false;
      if (position != null && !position.geometricallyEquals(other.position, epsilon))
         return false;
      if (linearVelocity != null && !linearVelocity.geometricallyEquals(other.linearVelocity, epsilon))
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
