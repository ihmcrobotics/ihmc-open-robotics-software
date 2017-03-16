package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomGeometry;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in taskspace. It holds the necessary information for one trajectory point. "
      + "Feel free to look at EuclideanTrajectoryPointMessage (translational) and EuclideanTrajectoryPointMessage (rotational)", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class SE3TrajectoryPointMessage extends Packet<SE3TrajectoryPointMessage> implements Transformable
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.")
   public Point3D position;
   @RosExportedField(documentation = "Define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.")
   public Quaternion orientation;
   @RosExportedField(documentation = "Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3D linearVelocity;
   @RosExportedField(documentation = "Define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3D angularVelocity;

   /**
    * Empty constructor for serialization.
    */
   public SE3TrajectoryPointMessage()
   {
   }

   public SE3TrajectoryPointMessage(Random random)
   {
      time = RandomNumbers.nextDoubleWithEdgeCases(random, 0.01);
      position = RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0);
      orientation = RandomGeometry.nextQuaternion(random);
      linearVelocity = RandomGeometry.nextVector3D(random);
      angularVelocity = RandomGeometry.nextVector3D(random);
   }

   public SE3TrajectoryPointMessage(SE3TrajectoryPointMessage se3TrajectoryPointMessage)
   {
      time = se3TrajectoryPointMessage.time;
      if (se3TrajectoryPointMessage.position != null)
         position = new Point3D(se3TrajectoryPointMessage.position);
      if (se3TrajectoryPointMessage.orientation != null)
         orientation = new Quaternion(se3TrajectoryPointMessage.orientation);
      if (se3TrajectoryPointMessage.linearVelocity != null)
         linearVelocity = new Vector3D(se3TrajectoryPointMessage.linearVelocity);
      if (se3TrajectoryPointMessage.angularVelocity != null)
         angularVelocity = new Vector3D(se3TrajectoryPointMessage.angularVelocity);
   }

   public SE3TrajectoryPointMessage(double time, Point3D position, Quaternion orientation, Vector3D linearVelocity, Vector3D angularVelocity)
   {
      this.time = time;
      this.position = new Point3D(position);
      this.orientation = new Quaternion(orientation);
      this.linearVelocity = new Vector3D(linearVelocity);
      this.angularVelocity = new Vector3D(angularVelocity);
   }

   //   @Override
   public void set(SE3TrajectoryPointMessage other)
   {
      time = other.time;
      if (other.position != null)
         position.set(other.position);
      else
         position.setToZero();
      if (other.orientation != null)
         orientation.set(other.orientation);
      else
         orientation.setToZero();
      if (other.linearVelocity != null)
         linearVelocity.set(other.linearVelocity);
      else
         linearVelocity.setToZero();
      if (other.angularVelocity != null)
         angularVelocity.set(other.angularVelocity);
      else
         angularVelocity.setToZero();
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public double getTime()
   {
      return time;
   }

   public void getPosition(Point3D positionToPack)
   {
      positionToPack.set(position);
   }

   public void setPosition(Point3D position)
   {
      this.position.set(position);
   }

   public void getOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void setOrientation(Quaternion orientation)
   {
      this.orientation.set(orientation);
   }

   public void getLinearVelocity(Vector3D linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void setLinearVelocity(Vector3D linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void getAngularVelocity(Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void setAngularVelocity(Vector3D angularVelocity)
   {
      this.angularVelocity.set(angularVelocity);
   }

   @Override
   public boolean epsilonEquals(SE3TrajectoryPointMessage other, double epsilon)
   {
      if (position == null && other.position != null)
         return false;
      if (position != null && other.position == null)
         return false;

      if (orientation == null && other.orientation != null)
         return false;
      if (orientation != null && other.orientation == null)
         return false;

      if (linearVelocity == null && other.linearVelocity != null)
         return false;
      if (linearVelocity != null && other.linearVelocity == null)
         return false;

      if (angularVelocity == null && other.angularVelocity != null)
         return false;
      if (angularVelocity != null && other.angularVelocity == null)
         return false;

      if (!MathTools.epsilonEquals(time, other.time, epsilon))
         return false;
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;

      return true;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
      transform.transform(orientation);
      transform.transform(linearVelocity);
      transform.transform(angularVelocity);
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
      String qxToString = doubleFormat.format(orientation.getX());
      String qyToString = doubleFormat.format(orientation.getY());
      String qzToString = doubleFormat.format(orientation.getZ());
      String qsToString = doubleFormat.format(orientation.getS());
      String wxToString = doubleFormat.format(angularVelocity.getX());
      String wyToString = doubleFormat.format(angularVelocity.getY());
      String wzToString = doubleFormat.format(angularVelocity.getZ());

      String timeToString = "time = " + doubleFormat.format(time);
      String positionToString = "position = (" + xToString + ", " + yToString + ", " + zToString + ")";
      String orientationToString = "orientation = (" + qxToString + ", " + qyToString + ", " + qzToString + ", " + qsToString + ")";
      String linearVelocityToString = "linear velocity = (" + xDotToString + ", " + yDotToString + ", " + zDotToString + ")";
      String angularVelocityToString = "angular velocity = (" + wxToString + ", " + wyToString + ", " + wzToString + ")";

      return "SE3 trajectory point: (" + timeToString + ", " + positionToString + ", " + orientationToString + ", " + linearVelocityToString + ", "
            + angularVelocityToString + ")";
   }
}
