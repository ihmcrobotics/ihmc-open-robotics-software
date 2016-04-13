package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;

@ClassDocumentation("This class is used to build trajectory messages in taskspace. It holds the only the translational information for one trajectory point (position & linear velocity). "
      + "Feel free to look at SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational)")
public class EuclideanTrajectoryPointMessage extends IHMCRosApiMessage<EuclideanTrajectoryPointMessage>
      implements EuclideanTrajectoryPointInterface<EuclideanTrajectoryPointMessage>, TransformableDataObject<EuclideanTrajectoryPointMessage>
{
   @FieldDocumentation("Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @FieldDocumentation("Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.")
   public Point3d position;
   @FieldDocumentation("Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.")
   public Vector3d linearVelocity;

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
         position = new Point3d(trajectoryPoint.position);
      if (trajectoryPoint.linearVelocity != null)
         linearVelocity = new Vector3d(trajectoryPoint.linearVelocity);
   }

   public EuclideanTrajectoryPointMessage(double time, Point3d position, Vector3d linearVelocity)
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
   public void getPosition(Point3d positionToPack)
   {
      positionToPack.set(position);
   }

   public void setPosition(Point3d position)
   {
      this.position = position;
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity = linearVelocity;
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
   public void applyTransform(RigidBodyTransform transform)
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
