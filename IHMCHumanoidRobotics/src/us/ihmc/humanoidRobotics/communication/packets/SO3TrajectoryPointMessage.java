package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosIgnoredField;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.random.RandomTools;

@RosMessagePacket(documentation =
      "This class is used to build trajectory messages in taskspace. It holds the only the rotational information for one trajectory point (orientation & angular velocity). "
      + "Feel free to look at EuclideanTrajectoryPointMessage (translational) and SE3TrajectoryPointMessage (rotational AND translational)",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class SO3TrajectoryPointMessage extends Packet<SO3TrajectoryPointMessage> implements Transformable
      //implements SO3TrajectoryPointInterface<SO3TrajectoryPointMessage>, TransformableDataObject<SO3TrajectoryPointMessage>
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.")
   public Quat4d orientation;
   @RosExportedField(documentation = "Define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3d angularVelocity;

   /**
    * Empty constructor for serialization.
    */
   public SO3TrajectoryPointMessage()
   {
   }

   public SO3TrajectoryPointMessage(Random random)
   {
      time = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      orientation = RandomTools.generateRandomQuaternion(random);
      angularVelocity = RandomTools.generateRandomVector(random);
   }

   public SO3TrajectoryPointMessage(SO3TrajectoryPointMessage trajectoryPoint)
   {
      time = trajectoryPoint.time;
      if (trajectoryPoint.orientation != null)
         orientation = new Quat4d(trajectoryPoint.orientation);
      if (trajectoryPoint.angularVelocity != null)
         angularVelocity = new Vector3d(trajectoryPoint.angularVelocity);
   }

   public SO3TrajectoryPointMessage(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      this.time = time;
      this.orientation = orientation;
      this.angularVelocity = angularVelocity;
   }

   public void set(SO3TrajectoryPointMessage other)
   {
      time = other.time;
      if (other.orientation != null)
         orientation.set(other.orientation);
      else
         orientation.set(0.0, 0.0, 0.0, 1.0);
      if (other.angularVelocity != null)
         angularVelocity.set(other.angularVelocity);
      else
         angularVelocity.set(0.0, 0.0, 0.0);
   }

   public double getTime()
   {
      return time;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public void getOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void setOrientation(Quat4d orientation)
   {
      this.orientation = orientation;
   }

   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity = angularVelocity;
   }

   public SO3TrajectoryPointMessage transform(RigidBodyTransform transform)
   {
      SO3TrajectoryPointMessage transformedTrajectoryPointMessage = new SO3TrajectoryPointMessage();

      transformedTrajectoryPointMessage.time = time;

      if (orientation != null)
         transformedTrajectoryPointMessage.orientation = TransformTools.getTransformedQuat(orientation, transform);
      else
         transformedTrajectoryPointMessage.orientation = null;

      if (angularVelocity != null)
         transformedTrajectoryPointMessage.angularVelocity = TransformTools.getTransformedVector(angularVelocity, transform);
      else
         transformedTrajectoryPointMessage.angularVelocity = null;

      return transformedTrajectoryPointMessage;
   }

   @Override
   public boolean epsilonEquals(SO3TrajectoryPointMessage other, double epsilon)
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

   @RosIgnoredField
   public Quat4d tempQuaternionForTransform;

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      if (tempQuaternionForTransform == null)
         tempQuaternionForTransform = new Quat4d();
      transform.getRotation(tempQuaternionForTransform);
      orientation.mul(tempQuaternionForTransform, orientation);
      transform.transform(angularVelocity);
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String qxToString = doubleFormat.format(orientation.getX());
      String qyToString = doubleFormat.format(orientation.getY());
      String qzToString = doubleFormat.format(orientation.getZ());
      String qsToString = doubleFormat.format(orientation.getW());
      String wxToString = doubleFormat.format(angularVelocity.getX());
      String wyToString = doubleFormat.format(angularVelocity.getY());
      String wzToString = doubleFormat.format(angularVelocity.getZ());

      String timeToString = "time = " + doubleFormat.format(time);
      String orientationToString = "orientation = (" + qxToString + ", " + qyToString + ", " + qzToString + ", " + qsToString + ")";
      String angularVelocityToString = "angular velocity = (" + wxToString + ", " + wyToString + ", " + wzToString + ")";

      return "SO3 trajectory point: (" + timeToString + ", " + orientationToString + ", " + angularVelocityToString + ")";
   }
}
