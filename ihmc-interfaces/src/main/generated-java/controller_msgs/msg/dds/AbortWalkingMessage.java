package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message is used to abort
 * walking, forcing the robot to switch back to double support and clear the footstep list.
 */
public class AbortWalkingMessage extends Packet<AbortWalkingMessage> implements Settable<AbortWalkingMessage>, EpsilonComparable<AbortWalkingMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;

   public AbortWalkingMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public AbortWalkingMessage(AbortWalkingMessage other)
   {
      this();
      set(other);
   }

   public void set(AbortWalkingMessage other)
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
   public boolean epsilonEquals(AbortWalkingMessage other, double epsilon)
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
      if (!(other instanceof AbortWalkingMessage))
         return false;

      AbortWalkingMessage otherMyClass = (AbortWalkingMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbortWalkingMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append("}");
      return builder.toString();
   }
}
