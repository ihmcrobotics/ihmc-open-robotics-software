package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

/**
 * This class is used to build trajectory messages in jointspace. It holds all the trajectory points to go through with a one-dimensional trajectory.
 * A third order polynomial function is used to interpolate between trajectory points.
 */

public abstract class Abstract1DTrajectoryMessage<T extends Abstract1DTrajectoryMessage<T>> extends Packet<T>
{
   @RosExportedField(documentation = "List of trajectory points to go through while executing the trajectory.")
   public TrajectoryPoint1DMessage[] trajectoryPoints;

   /**
    * Empty constructor for serialization.
    */
   public Abstract1DTrajectoryMessage()
   {
   }

   public Abstract1DTrajectoryMessage(Abstract1DTrajectoryMessage<T> trajectory1dMessage)
   {
      trajectoryPoints = new TrajectoryPoint1DMessage[trajectory1dMessage.getNumberOfTrajectoryPoints()];
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         trajectoryPoints[i] = new TrajectoryPoint1DMessage(trajectory1dMessage.getTrajectoryPoint(i));
      }
   }

   /**
    * Use this constructor to go straight to the given end point.
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public Abstract1DTrajectoryMessage(double trajectoryTime, double desiredPosition)
   {
      trajectoryPoints = new TrajectoryPoint1DMessage[] {new TrajectoryPoint1DMessage(trajectoryTime, desiredPosition, 0.0)};
   }
   
   public Abstract1DTrajectoryMessage(SimpleTrajectoryPoint1DList trajectoryData)
   {
      int numberOfPoints = trajectoryData.getNumberOfTrajectoryPoints();
      trajectoryPoints = new TrajectoryPoint1DMessage[numberOfPoints];

      for (int i=0; i<numberOfPoints; i++)
      {
         SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(i);
         trajectoryPoints[i] = new TrajectoryPoint1DMessage(trajectoryPoint);
         
      }
   }
   
   public void getTrajectoryPoints(SimpleTrajectoryPoint1DList trajectoryPointListToPack)
   {
      trajectoryPointListToPack.clear();
      
      TrajectoryPoint1DMessage[] trajectoryPointMessages = getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.length;

      for (int i=0; i<numberOfPoints; i++)
      {
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPointMessages[i];
         trajectoryPointListToPack.addTrajectoryPoint(trajectoryPoint1DMessage.time, trajectoryPoint1DMessage.position, trajectoryPoint1DMessage.velocity);
      }
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public Abstract1DTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      trajectoryPoints = new TrajectoryPoint1DMessage[numberOfTrajectoryPoints];
   }

   /**
    * Create a trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 1D position to be reached at this trajectory point.
    * @param velocity define the desired 1D velocity to be reached at this trajectory point.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, double position, double velocity)
   {
      rangeCheck(trajectoryPointIndex);
      trajectoryPoints[trajectoryPointIndex] = new TrajectoryPoint1DMessage(time, position, velocity);
   }

   public void set(T other)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         throw new RuntimeException("Trajectory messages do not have the same number of trajectory points.");

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints[i].set(other.trajectoryPoints[i]);
   }

   public final int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.length;
   }

   public final TrajectoryPoint1DMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints[trajectoryPointIndex];
   }

   public final TrajectoryPoint1DMessage[] getTrajectoryPoints()
   {
      return trajectoryPoints;
   }

   public final TrajectoryPoint1DMessage getLastTrajectoryPoint()
   {
      return trajectoryPoints[getNumberOfTrajectoryPoints() - 1];
   }

   public final double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().getTime();
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException(
               "Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: " + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!trajectoryPoints[i].epsilonEquals(other.trajectoryPoints[i], epsilon))
            return false;
      }

      return true;
   }

   public Abstract1DTrajectoryMessage(Random random)
   {
      this(random.nextInt(16) + 1);

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         trajectoryPoints[i] = new TrajectoryPoint1DMessage(random);
      }
   }

   @Override
   public String toString()
   {
      if (trajectoryPoints != null)
         return "Trajectory 1D: number of 1D trajectory points = " + getNumberOfTrajectoryPoints() + ".";
      else
         return "Trajectory 1D: no 1D trajectory point.";
   }
}
