package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

@RosMessagePacket(documentation = "This class is used to build 1D trajectory messages including jointspace trajectory messages."
      + " For 3D trajectory points look at EuclideanTrajectoryMessage (translational), SO3TrajectoryPointMessage (rotational), and SE3TrajectoryPointMessage (translational AND rotational).",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class TrajectoryPoint1DMessage extends Packet<TrajectoryPoint1DMessage> implements OneDoFTrajectoryPointInterface<TrajectoryPoint1DMessage>
{
   @RosExportedField(documentation = "Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.")
   public double time;
   @RosExportedField(documentation = "Define the desired 1D position to be reached at this trajectory point.")
   public double position;
   @RosExportedField(documentation = "Define the desired 1D velocity to be reached at this trajectory point.")
   public double velocity;

   /**
    * Empty constructor for serialization.
    */
   public TrajectoryPoint1DMessage()
   {
   }

   public TrajectoryPoint1DMessage(Random random)
   {
      time = RandomNumbers.nextDoubleWithEdgeCases(random, 0.01);
      position = RandomNumbers.nextDoubleWithEdgeCases(random, 0.01);
      velocity = RandomNumbers.nextDoubleWithEdgeCases(random, 0.01);
   }

   public TrajectoryPoint1DMessage(OneDoFTrajectoryPointInterface<?> trajectoryPoint)
   {
      time = trajectoryPoint.getTime();
      position = trajectoryPoint.getPosition();
      velocity = trajectoryPoint.getVelocity();
   }

   public TrajectoryPoint1DMessage(double time, double position, double velocity)
   {
      this.time = time;
      this.position = position;
      this.velocity = velocity;
   }

   @Override
   public void set(TrajectoryPoint1DMessage other)
   {
      time = other.time;
      position = other.position;
      velocity = other.velocity;
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
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
   public double getTime()
   {
      return time;
   }

   @Override
   public double getPosition()
   {
      return position;
   }

   public void setPosition(double position)
   {
      this.position = position;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public void setTimeToZero()
   {
      time = 0.0;
   }

   @Override
   public void setToZero()
   {
      time = 0.0;
      position = 0.0;
      velocity = 0.0;
   }

   @Override
   public void setTimeToNaN()
   {
      time = Double.NaN;
   }

   @Override
   public void setToNaN()
   {
      time = Double.NaN;
      position = Double.NaN;
      velocity = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(time) || Double.isNaN(position) || Double.isNaN(velocity);
   }

   @Override
   public boolean epsilonEquals(TrajectoryPoint1DMessage other, double epsilon)
   {
      if (!MathTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(getPosition(), other.getPosition(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeString = "time = " + doubleFormat.format(getTime());
      String positionString = "position = " + doubleFormat.format(getPosition());
      String velocityString = "velocity = " + doubleFormat.format(getVelocity());
      return "Trajectory point 1D: (" + timeString + ", " + positionString + ", " + velocityString + ")";
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Do nothing since simple numbers here.
   }
}
