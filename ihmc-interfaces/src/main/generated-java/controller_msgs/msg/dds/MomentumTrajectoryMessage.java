package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message can be used to send a
 * predefined angular momentum trajectory to the controller. This trajectory will be used for ICP
 * planning.
 */
public class MomentumTrajectoryMessage extends Packet<MomentumTrajectoryMessage>
      implements Settable<MomentumTrajectoryMessage>, EpsilonComparable<MomentumTrajectoryMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * List of angular momentum trajectory waypoints. Each waypoint contains the angular momentum and
    * the angular momentum rate at a given time. NOTE: As the angular momentum can't be encoded
    * using the SO3TrajectoryMessage, the EuclideanTrajectoryMessage is used.
    */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage angular_momentum_trajectory_;

   public MomentumTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      angular_momentum_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();
   }

   public MomentumTrajectoryMessage(MomentumTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(MomentumTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.angular_momentum_trajectory_, angular_momentum_trajectory_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * List of angular momentum trajectory waypoints. Each waypoint contains the angular momentum and
    * the angular momentum rate at a given time. NOTE: As the angular momentum can't be encoded
    * using the SO3TrajectoryMessage, the EuclideanTrajectoryMessage is used.
    */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getAngularMomentumTrajectory()
   {
      return angular_momentum_trajectory_;
   }

   @Override
   public boolean epsilonEquals(MomentumTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.angular_momentum_trajectory_.epsilonEquals(other.angular_momentum_trajectory_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof MomentumTrajectoryMessage))
         return false;

      MomentumTrajectoryMessage otherMyClass = (MomentumTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.angular_momentum_trajectory_.equals(otherMyClass.angular_momentum_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MomentumTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("angular_momentum_trajectory=");
      builder.append(this.angular_momentum_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
