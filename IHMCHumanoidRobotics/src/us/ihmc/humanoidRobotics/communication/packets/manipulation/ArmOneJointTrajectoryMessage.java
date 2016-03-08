package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.humanoidRobotics.communication.packets.Abstract1DTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;

@ClassDocumentation("This class is used to build trajectory messages in jointspace. It holds all the trajectory points to go through with a one-dimensional trajectory."
      + " A third order polynomial function is used to interpolate between trajectory points.")
public class ArmOneJointTrajectoryMessage extends Abstract1DTrajectoryMessage<ArmOneJointTrajectoryMessage>
{
   /**
    * Empty constructor for serialization.
    */
   public ArmOneJointTrajectoryMessage()
   {
      super();
   }

   public ArmOneJointTrajectoryMessage(ArmOneJointTrajectoryMessage trajectory1dMessage)
   {
      super(trajectory1dMessage);
   }

//   public ArmOneJointTrajectoryMessage(TrajectoryPointListInterface<?, ? extends OneDoFTrajectoryPointInterface<?>> trajectory1dMessage)
//   {
//      super(trajectory1dMessage);
//   }

   /**
    * Use this constructor to go straight to the given end point.
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public ArmOneJointTrajectoryMessage(double trajectoryTime, double desiredPosition)
   {
      super(trajectoryTime, desiredPosition);
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public ArmOneJointTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
   }

   @Override
   public void set(ArmOneJointTrajectoryMessage other)
   {
      super.set(other);
   }

   @Override
   public boolean epsilonEquals(ArmOneJointTrajectoryMessage other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }

   public ArmOneJointTrajectoryMessage(Random random)
   {
      super(random);
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
