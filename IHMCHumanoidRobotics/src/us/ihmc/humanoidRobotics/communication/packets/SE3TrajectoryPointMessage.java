package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosIgnoredField;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.random.RandomTools;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in taskspace. It holds the necessary information for one trajectory point. "
      + "Feel free to look at EuclideanTrajectoryPointMessage (translational) and EuclideanTrajectoryPointMessage (rotational)",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class SE3TrajectoryPointMessage extends Packet<SE3TrajectoryPointMessage> implements Transformable
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.")
   public Point3d position;
   @RosExportedField(documentation = "Define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.")
   public Quat4d orientation;
   @RosExportedField(documentation = "Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3d linearVelocity;
   @RosExportedField(documentation = "Define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3d angularVelocity;

   /**
    * Empty constructor for serialization.
    */
   public SE3TrajectoryPointMessage()
   {
   }

   public SE3TrajectoryPointMessage(Random random)
   {
      time = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      position = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
      orientation = RandomTools.generateRandomQuaternion(random);
      linearVelocity = RandomTools.generateRandomVector(random);
      angularVelocity = RandomTools.generateRandomVector(random);
   }

   public SE3TrajectoryPointMessage(SE3TrajectoryPointMessage se3TrajectoryPointMessage)
   {
      time = se3TrajectoryPointMessage.time;
      if (se3TrajectoryPointMessage.position != null)
         position = new Point3d(se3TrajectoryPointMessage.position);
      if (se3TrajectoryPointMessage.orientation != null)
         orientation = new Quat4d(se3TrajectoryPointMessage.orientation);
      if (se3TrajectoryPointMessage.linearVelocity != null)
         linearVelocity = new Vector3d(se3TrajectoryPointMessage.linearVelocity);
      if (se3TrajectoryPointMessage.angularVelocity != null)
         angularVelocity = new Vector3d(se3TrajectoryPointMessage.angularVelocity);
   }

   public SE3TrajectoryPointMessage(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      this.time = time;
      this.position = position;
      this.orientation = orientation;
      this.linearVelocity = linearVelocity;
      this.angularVelocity = angularVelocity;
   }

//   @Override
   public void set(SE3TrajectoryPointMessage other)
   {
      time = other.time;
      if (other.position != null)
         position.set(other.position);
      else
         position.set(0.0, 0.0, 0.0);
      if (other.orientation != null)
         orientation.set(other.orientation);
      else
         orientation.set(0.0, 0.0, 0.0, 1.0);
      if (other.linearVelocity != null)
         linearVelocity.set(other.linearVelocity);
      else
         linearVelocity.set(0.0, 0.0, 0.0);
      if (other.angularVelocity != null)
         angularVelocity.set(other.angularVelocity);
      else
         angularVelocity.set(0.0, 0.0, 0.0);
   }

//   @Override
//   public void addTimeOffset(double timeOffsetToAdd)
//   {
//      time += timeOffsetToAdd;
//   }
//
//   @Override
//   public void subtractTimeOffset(double timeOffsetToSubtract)
//   {
//      time -= timeOffsetToSubtract;
//   }
//
//   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

//   @Override
//   public double positionDistance(SE3TrajectoryPointMessage other)
//   {
//      return position.distance(other.position);
//   }
//
//   @Override
   public double getTime()
   {
      return time;
   }

//   @Override
   public void getPosition(Point3d positionToPack)
   {
      positionToPack.set(position);
   }

//   @Override
   public void setPosition(Point3d position)
   {
      this.position = position;
   }

//   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientationToPack.set(orientation);
   }

//   @Override
   public void setOrientation(Quat4d orientation)
   {
      this.orientation = orientation;
   }

//   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

//   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity = linearVelocity;
   }

//   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

//   @Override
   public void setAngularVelocity(Vector3d angularVelocity)
   {
      this.angularVelocity = angularVelocity;
   }

//   @Override
//   public void setTimeToZero()
//   {
//      time = 0.0;
//   }
//
//   @Override
//   public void setPositionToZero()
//   {
//      position.set(0.0, 0.0, 0.0);
//   }
//
//   @Override
//   public void setOrientationToZero()
//   {
//      orientation.set(0.0, 0.0, 0.0, 1.0);
//   }
//
//   @Override
//   public void setLinearVelocityToZero()
//   {
//      linearVelocity.set(0.0, 0.0, 0.0);
//   }
//
//   @Override
//   public void setAngularVelocityToZero()
//   {
//      angularVelocity.set(0.0, 0.0, 0.0);
//   }
//
//   @Override
//   public void setToZero()
//   {
//      setTimeToZero();
//      setPositionToZero();
//      setOrientationToZero();
//      setLinearVelocityToZero();
//      setAngularVelocityToZero();
//   }
//
//   @Override
//   public void setTimeToNaN()
//   {
//      time = Double.NaN;
//   }
//
//   @Override
//   public void setPositionToNaN()
//   {
//      position.set(Double.NaN, Double.NaN, Double.NaN);
//   }
//
//   @Override
//   public void setOrientationToNaN()
//   {
//      orientation.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
//   }
//
//   @Override
//   public void setLinearVelocityToNaN()
//   {
//      linearVelocity.set(Double.NaN, Double.NaN, Double.NaN);
//   }
//
//   @Override
//   public void setAngularVelocityToNaN()
//   {
//      angularVelocity.set(Double.NaN, Double.NaN, Double.NaN);
//   }
//
//   @Override
//   public void setToNaN()
//   {
//      setTimeToNaN();
//      setPositionToNaN();
//      setOrientationToNaN();
//      setLinearVelocityToNaN();
//      setAngularVelocityToNaN();
//   }
//
//   @Override
//   public boolean containsNaN()
//   {
//      if (Double.isNaN(time))
//         return true;
//      if (Double.isNaN(position.getX()) || Double.isNaN(position.getY()) || Double.isNaN(position.getZ()))
//         return true;
//      if (Double.isNaN(orientation.getX()) || Double.isNaN(orientation.getY()) || Double.isNaN(orientation.getZ()) || Double.isNaN(orientation.getW()))
//         return true;
//      if (Double.isNaN(linearVelocity.getX()) || Double.isNaN(linearVelocity.getY()) || Double.isNaN(linearVelocity.getZ()))
//         return true;
//      if (Double.isNaN(angularVelocity.getX()) || Double.isNaN(angularVelocity.getY()) || Double.isNaN(angularVelocity.getZ()))
//         return true;
//      return false;
//   }

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

//   @Override
//   public SE3TrajectoryPointMessage transform(RigidBodyTransform transform)
//   {
//      SE3TrajectoryPointMessage transformedTrajectoryPointMessage = new SE3TrajectoryPointMessage();
//
//      transformedTrajectoryPointMessage.time = time;
//
//      if (position != null)
//         transformedTrajectoryPointMessage.position = TransformTools.getTransformedPoint(position, transform);
//      else
//         transformedTrajectoryPointMessage.position = null;
//
//      if (orientation != null)
//         transformedTrajectoryPointMessage.orientation = TransformTools.getTransformedQuat(orientation, transform);
//      else
//         transformedTrajectoryPointMessage.orientation = null;
//
//      if (linearVelocity != null)
//         transformedTrajectoryPointMessage.linearVelocity = TransformTools.getTransformedVector(linearVelocity, transform);
//      else
//         transformedTrajectoryPointMessage.linearVelocity = null;
//
//      if (angularVelocity != null)
//         transformedTrajectoryPointMessage.angularVelocity = TransformTools.getTransformedVector(angularVelocity, transform);
//      else
//         transformedTrajectoryPointMessage.angularVelocity = null;
//
//      return transformedTrajectoryPointMessage;
//   }

   @RosIgnoredField
   public Quat4d tempQuaternionForTransform;

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(position);
      if (tempQuaternionForTransform == null)
         tempQuaternionForTransform = new Quat4d();
      transform.getRotation(tempQuaternionForTransform);
      orientation.mul(tempQuaternionForTransform, orientation);
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
      String qsToString = doubleFormat.format(orientation.getW());
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
