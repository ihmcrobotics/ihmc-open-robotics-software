package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. Stop the execution of any trajectory
 * being executed.
 */
public class StopAllTrajectoryMessage extends Packet<StopAllTrajectoryMessage>
      implements Settable<StopAllTrajectoryMessage>, EpsilonComparable<StopAllTrajectoryMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;

   public StopAllTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public StopAllTrajectoryMessage(StopAllTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(StopAllTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   @Override
   public boolean epsilonEquals(StopAllTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
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
      if (!(other instanceof StopAllTrajectoryMessage))
         return false;

      StopAllTrajectoryMessage otherMyClass = (StopAllTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StopAllTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append("}");
      return builder.toString();
   }
}
