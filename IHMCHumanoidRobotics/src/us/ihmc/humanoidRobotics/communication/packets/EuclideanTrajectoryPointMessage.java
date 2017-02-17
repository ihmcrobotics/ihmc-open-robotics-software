package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in taskspace. It holds the only the translational information for one trajectory point (position & linear velocity). "
      + "Feel free to look at SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational)", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class EuclideanTrajectoryPointMessage extends Packet<EuclideanTrajectoryPointMessage>
      implements EuclideanTrajectoryPointInterface<EuclideanTrajectoryPointMessage>, TransformableDataObject<EuclideanTrajectoryPointMessage>
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

   public EuclideanTrajectoryPointMessage(Random random)
   {
      time = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      position = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
      linearVelocity = RandomTools.generateRandomVector(random);
   }

   public EuclideanTrajectoryPointMessage(EuclideanTrajectoryPointMessage trajectoryPoint)
   {
      time = trajectoryPoint.time;
      if (trajectoryPoint.position != null)
         position = new Point3D(trajectoryPoint.position);
      if (trajectoryPoint.linearVelocity != null)
         linearVelocity = new Vector3D(trajectoryPoint.linearVelocity);
   }

   public EuclideanTrajectoryPointMessage(double time, Point3D position, Vector3D linearVelocity)
   {
      this.time = time;
      this.position = position;
      this.linearVelocity = linearVelocity;
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
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time += timeOffsetToAdd;
   }

   @Override
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time -= timeOffsetToSubtract;
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      if (this.position == null)
         this.position = new Point3D(position);
      else
         this.position.set(position);
   }

   @Override
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

   @Override
   public void setTimeToZero()
   {
      time = 0.0;
   }

   @Override
   public void setPositionToZero()
   {
      position.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setLinearVelocityToZero()
   {
      linearVelocity.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToZero()
   {
      setTimeToZero();
      setPositionToZero();
      setLinearVelocityToZero();
   }

   @Override
   public void setTimeToNaN()
   {
      time = Double.NaN;
   }

   @Override
   public void setPositionToNaN()
   {
      position.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      linearVelocity.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public void setToNaN()
   {
      setTimeToNaN();
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   @Override
   public double positionDistance(EuclideanTrajectoryPointMessage other)
   {
      return position.distance(other.position);
   }

   @Override
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
   public EuclideanTrajectoryPointMessage transform(RigidBodyTransform transform)
   {
      EuclideanTrajectoryPointMessage transformedTrajectoryPointMessage = new EuclideanTrajectoryPointMessage();

      transformedTrajectoryPointMessage.time = time;

      if (position != null)
         transformedTrajectoryPointMessage.position = TransformTools.getTransformedPoint(position, transform);
      else
         transformedTrajectoryPointMessage.position = null;

      if (linearVelocity != null)
         transformedTrajectoryPointMessage.linearVelocity = TransformTools.getTransformedVector(linearVelocity, transform);
      else
         transformedTrajectoryPointMessage.linearVelocity = null;

      return transformedTrajectoryPointMessage;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
      transform.transform(linearVelocity);
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
