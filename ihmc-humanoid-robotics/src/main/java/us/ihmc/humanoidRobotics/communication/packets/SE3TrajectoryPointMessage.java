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
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in taskspace. It holds the necessary information for one trajectory point. "
      + "Feel free to look at EuclideanTrajectoryPointMessage (translational) and EuclideanTrajectoryPointMessage (rotational)", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class SE3TrajectoryPointMessage extends Packet<SE3TrajectoryPointMessage>
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 3D position to be reached at this trajectory point.")
   public Point3D position = new Point3D();
   @RosExportedField(documentation = "Define the desired 3D orientation to be reached at this trajectory point.")
   public Quaternion orientation = new Quaternion();
   @RosExportedField(documentation = "Define the desired 3D linear velocity to be reached at this trajectory point.")
   public Vector3D linearVelocity = new Vector3D();
   @RosExportedField(documentation = "Define the desired 3D angular velocity to be reached at this trajectory point.")
   public Vector3D angularVelocity = new Vector3D();

   /**
    * Empty constructor for serialization.
    */
   public SE3TrajectoryPointMessage()
   {
      position = new Point3D();
      orientation = new Quaternion();
      linearVelocity = new Vector3D();
      angularVelocity = new Vector3D();
   }

   public SE3TrajectoryPointMessage(SE3TrajectoryPointMessage other)
   {
      time = other.time;
      if (other.position != null)
         position = new Point3D(other.position);
      if (other.orientation != null)
         orientation = new Quaternion(other.orientation);
      if (other.linearVelocity != null)
         linearVelocity = new Vector3D(other.linearVelocity);
      if (other.angularVelocity != null)
         angularVelocity = new Vector3D(other.angularVelocity);
   }

   @Override
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
      setPacketInformation(other);
   }

   public void packData(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      trajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public double getTime()
   {
      return time;
   }

   public void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   public void setPosition(Point3DReadOnly position)
   {
      this.position.set(position);
   }

   public void getOrientation(QuaternionBasics orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void setOrientation(QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void getAngularVelocity(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void setAngularVelocity(Vector3DReadOnly angularVelocity)
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

      if (!MathTools.epsilonCompare(time, other.time, epsilon))
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
