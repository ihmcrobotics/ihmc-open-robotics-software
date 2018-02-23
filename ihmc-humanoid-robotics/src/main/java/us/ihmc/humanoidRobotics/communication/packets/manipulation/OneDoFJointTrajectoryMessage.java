package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.idl.PreallocatedList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in jointspace. It holds all the trajectory points to go through with a one-dimensional trajectory."
      + " A third order polynomial function is used to interpolate between trajectory points.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class OneDoFJointTrajectoryMessage extends Packet<OneDoFJointTrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points to go through while executing the trajectory.")
   public PreallocatedList<TrajectoryPoint1DMessage> trajectoryPoints = new PreallocatedList<>(TrajectoryPoint1DMessage.class, TrajectoryPoint1DMessage::new,
                                                                                               2000);
   @RosExportedField(documentation = "QP Weight, if Too low, in the event the qp can't achieve all of the objectives it may stop trying to achieve the desireds, if too high, it will favor this joint over other objectives. If set to NaN it will use the default weight for that joint")
   public double weight = Double.NaN;

   /**
    * Empty constructor for serialization.
    */
   public OneDoFJointTrajectoryMessage()
   {
      super();
   }

   public OneDoFJointTrajectoryMessage(OneDoFJointTrajectoryMessage other)
   {
      MessageTools.copyData(other.trajectoryPoints, trajectoryPoints);
      weight = other.getWeight();
   }

   @Override
   public void set(OneDoFJointTrajectoryMessage other)
   {
      MessageTools.copyData(other.trajectoryPoints, trajectoryPoints);
      weight = other.weight;
      setPacketInformation(other);
   }

   /**
    * Create a trajectory point.
    * 
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when
    *           the trajectory starts.
    * @param position define the desired 1D position to be reached at this trajectory point.
    * @param velocity define the desired 1D velocity to be reached at this trajectory point.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, double position, double velocity)
   {
      rangeCheck(trajectoryPointIndex);
      trajectoryPoints.get(trajectoryPointIndex).set(HumanoidMessageTools.createTrajectoryPoint1DMessage(time, position, velocity));
   }

   public void getTrajectoryPoints(SimpleTrajectoryPoint1DList trajectoryPointListToPack)
   {
      trajectoryPointListToPack.clear();

      PreallocatedList<TrajectoryPoint1DMessage> trajectoryPointMessages = getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPointMessages.get(i);
         trajectoryPointListToPack.addTrajectoryPoint(trajectoryPoint1DMessage.time, trajectoryPoint1DMessage.position, trajectoryPoint1DMessage.velocity);
      }
   }

   public final int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   public final TrajectoryPoint1DMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   public final PreallocatedList<TrajectoryPoint1DMessage> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }

   public final TrajectoryPoint1DMessage getLastTrajectoryPoint()
   {
      return trajectoryPoints.get(getNumberOfTrajectoryPoints() - 1);
   }

   public final double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().getTime();
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException("Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: "
               + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(OneDoFJointTrajectoryMessage other, double epsilon)
   {
      if (!MessageTools.epsilonEquals(trajectoryPoints, other.trajectoryPoints, epsilon))
         return false;

      if (!Double.isNaN(weight) || !Double.isNaN(other.weight))
      {
         if (!MathTools.epsilonEquals(weight, other.weight, 0.0003))
            return false;
      }
      return true;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public double getWeight()
   {
      return weight;
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
