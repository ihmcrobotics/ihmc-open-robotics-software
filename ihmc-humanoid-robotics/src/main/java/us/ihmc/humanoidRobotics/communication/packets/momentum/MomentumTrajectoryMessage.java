package us.ihmc.humanoidRobotics.communication.packets.momentum;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryMessage;

/**
 * This message can be used to send a predefined angular momentum trajectory to the controller. This trajectory
 * will be used for ICP planning.
 */
public class MomentumTrajectoryMessage extends Packet<MomentumTrajectoryMessage>
{
   /**
    * List of angular momentum trajectory waypoints. Each waypoint contains the angular momentum and the
    * angular momentum rate at a given time.
    */
   public EuclideanTrajectoryMessage angularMomentumTrajectory;

   public MomentumTrajectoryMessage()
   {
      angularMomentumTrajectory = new EuclideanTrajectoryMessage();
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public MomentumTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      angularMomentumTrajectory = new EuclideanTrajectoryMessage(numberOfTrajectoryPoints);
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(MomentumTrajectoryMessage other)
   {
      angularMomentumTrajectory = new EuclideanTrajectoryMessage();
      angularMomentumTrajectory.set(other.angularMomentumTrajectory);
      setPacketInformation(other);
   }

   public EuclideanTrajectoryMessage getAngularMomentumTrajectory()
   {
      return angularMomentumTrajectory;
   }

   @Override
   public boolean epsilonEquals(MomentumTrajectoryMessage other, double epsilon)
   {
      return angularMomentumTrajectory.epsilonEquals(other.angularMomentumTrajectory, epsilon);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (angularMomentumTrajectory != null)
         angularMomentumTrajectory.setUniqueId(uniqueId);
   }
}
