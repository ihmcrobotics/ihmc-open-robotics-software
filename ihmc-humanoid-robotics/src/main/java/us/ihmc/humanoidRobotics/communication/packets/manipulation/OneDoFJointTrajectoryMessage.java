package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.idl.RecyclingArrayListPubSub;

@RosMessagePacket(documentation = "This class is used to build trajectory messages in jointspace. It holds all the trajectory points to go through with a one-dimensional trajectory."
      + " A third order polynomial function is used to interpolate between trajectory points.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class OneDoFJointTrajectoryMessage extends Packet<OneDoFJointTrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points to go through while executing the trajectory.")
   public RecyclingArrayListPubSub<TrajectoryPoint1DMessage> trajectoryPoints = new RecyclingArrayListPubSub<>(TrajectoryPoint1DMessage.class,
                                                                                                       TrajectoryPoint1DMessage::new, 5);
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

   public final RecyclingArrayListPubSub<TrajectoryPoint1DMessage> getTrajectoryPoints()
   {
      return trajectoryPoints;
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
         return "Trajectory 1D: number of 1D trajectory points = " + trajectoryPoints.size() + ".";
      else
         return "Trajectory 1D: no 1D trajectory point.";
   }

}
