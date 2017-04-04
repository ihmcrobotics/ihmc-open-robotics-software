package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.Abstract1DTrajectoryMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

@RosMessagePacket(documentation =
      "This class is used to build trajectory messages in jointspace. It holds all the trajectory points to go through with a one-dimensional trajectory."
      + " A third order polynomial function is used to interpolate between trajectory points.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class OneDoFJointTrajectoryMessage extends Abstract1DTrajectoryMessage<OneDoFJointTrajectoryMessage>
{
   @RosExportedField(documentation = "QP Weight, if Too low, in the event the qp can't achieve all of the objectives it may stop trying to achieve the desireds, if too high, it will favor this joint over other objectives. If set to NaN it will use the default weight for that joint")
   public double weight = Double.NaN;
   
   /**
    * Empty constructor for serialization.
    */
   public OneDoFJointTrajectoryMessage()
   {
      super();
   }

   public OneDoFJointTrajectoryMessage(OneDoFJointTrajectoryMessage trajectory1dMessage)
   {
      super(trajectory1dMessage);
   }

   public OneDoFJointTrajectoryMessage(SimpleTrajectoryPoint1DList trajectoryData)
   {
      super(trajectoryData);
   }

   /**
    * Use this constructor to go straight to the given end point.
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    */
   public OneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition)
   {
      super(trajectoryTime, desiredPosition);
   }
   
   /**
    * Use this constructor to go straight to the given end point.
    * @param trajectoryTime how long it takes to reach the desired position.
    * @param desiredPosition desired end point position.
    * @param weight the weight for the qp
    */
   public OneDoFJointTrajectoryMessage(double trajectoryTime, double desiredPosition, double weight)
   {
      super(trajectoryTime, desiredPosition);
      this.weight = weight;
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, double, double)} for each trajectory point afterwards.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public OneDoFJointTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
   }

   @Override
   public void set(OneDoFJointTrajectoryMessage other)
   {
      super.set(other);
      weight = other.weight;
   }

   @Override
   public boolean epsilonEquals(OneDoFJointTrajectoryMessage other, double epsilon)
   {
      boolean equals = super.epsilonEquals(other, epsilon);
      if(!Double.isNaN(weight) || !Double.isNaN(other.weight))
      {
         equals &= MathTools.epsilonEquals(weight, other.weight, 0.0003);
      }
      return equals;
   }
   
   public void setWeight(double weight)
   {
      this.weight = weight;
   }
   
   public double getWeight()
   {
      return weight;
   }

   public OneDoFJointTrajectoryMessage(Random random)
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
