package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This class is used to build 1D trajectory messages including jointspace trajectory messages."
      + " For 3D trajectory points look at EuclideanTrajectoryMessage (translational), SO3TrajectoryPointMessage (rotational), and SE3TrajectoryPointMessage (translational AND rotational).", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class TrajectoryPoint1DMessage extends Packet<TrajectoryPoint1DMessage>
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

   public TrajectoryPoint1DMessage(TrajectoryPoint1DMessage trajectoryPoint)
   {
      time = trajectoryPoint.getTime();
      position = trajectoryPoint.getPosition();
      velocity = trajectoryPoint.getVelocity();
   }

   @Override
   public void set(TrajectoryPoint1DMessage other)
   {
      time = other.time;
      position = other.position;
      velocity = other.velocity;
      setPacketInformation(other);
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public double getTime()
   {
      return time;
   }

   public double getPosition()
   {
      return position;
   }

   public void setPosition(double position)
   {
      this.position = position;
   }

   public double getVelocity()
   {
      return velocity;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public boolean epsilonEquals(TrajectoryPoint1DMessage other, double epsilon)
   {
      if (!MathTools.epsilonCompare(getTime(), other.getTime(), epsilon))
         return false;
      if (!MathTools.epsilonCompare(getPosition(), other.getPosition(), epsilon))
         return false;
      if (!MathTools.epsilonCompare(getVelocity(), other.getVelocity(), epsilon))
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
}
