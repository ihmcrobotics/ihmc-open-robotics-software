package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message commands the controller
 * to move in taskspace the chest to the desired orientation while going through the specified
 * trajectory points. A Hermite based curve (third order) is used to interpolate the orientations.
 * To execute a simple trajectory to reach a desired chest orientation, set only one trajectory
 * point with zero velocity and its time to be equal to the desired trajectory time.
 */
public class ChestTrajectoryMessage extends Packet<ChestTrajectoryMessage>
      implements Settable<ChestTrajectoryMessage>, EpsilonComparable<ChestTrajectoryMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * The orientation trajectory information.
    */
   public controller_msgs.msg.dds.SO3TrajectoryMessage so3_trajectory_;

   public ChestTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      so3_trajectory_ = new controller_msgs.msg.dds.SO3TrajectoryMessage();
   }

   public ChestTrajectoryMessage(ChestTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(ChestTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.staticCopy(other.so3_trajectory_, so3_trajectory_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * The orientation trajectory information.
    */
   public controller_msgs.msg.dds.SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3_trajectory_;
   }

   @Override
   public boolean epsilonEquals(ChestTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.so3_trajectory_.epsilonEquals(other.so3_trajectory_, epsilon))
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
      if (!(other instanceof ChestTrajectoryMessage))
         return false;

      ChestTrajectoryMessage otherMyClass = (ChestTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.so3_trajectory_.equals(otherMyClass.so3_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChestTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("so3_trajectory=");
      builder.append(this.so3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
