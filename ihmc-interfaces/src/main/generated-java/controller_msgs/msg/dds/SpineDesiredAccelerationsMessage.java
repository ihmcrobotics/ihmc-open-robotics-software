package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message gives the user the
 * option to bypass IHMC feedback controllers for the spine joints by sending desired spine joint
 * accelerations. One needs experience in control when activating the bypass as it can result in
 * unexpected behaviors for unreasonable accelerations.
 */
public class SpineDesiredAccelerationsMessage extends Packet<SpineDesiredAccelerationsMessage>
      implements Settable<SpineDesiredAccelerationsMessage>, EpsilonComparable<SpineDesiredAccelerationsMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * The desired joint acceleration information.
    */
   public controller_msgs.msg.dds.DesiredAccelerationsMessage desired_accelerations_;

   public SpineDesiredAccelerationsMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      desired_accelerations_ = new controller_msgs.msg.dds.DesiredAccelerationsMessage();
   }

   public SpineDesiredAccelerationsMessage(SpineDesiredAccelerationsMessage other)
   {
      this();
      set(other);
   }

   public void set(SpineDesiredAccelerationsMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.staticCopy(other.desired_accelerations_, desired_accelerations_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * The desired joint acceleration information.
    */
   public controller_msgs.msg.dds.DesiredAccelerationsMessage getDesiredAccelerations()
   {
      return desired_accelerations_;
   }

   @Override
   public boolean epsilonEquals(SpineDesiredAccelerationsMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.desired_accelerations_.epsilonEquals(other.desired_accelerations_, epsilon))
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
      if (!(other instanceof SpineDesiredAccelerationsMessage))
         return false;

      SpineDesiredAccelerationsMessage otherMyClass = (SpineDesiredAccelerationsMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.desired_accelerations_.equals(otherMyClass.desired_accelerations_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SpineDesiredAccelerationsMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("desired_accelerations=");
      builder.append(this.desired_accelerations_);
      builder.append("}");
      return builder.toString();
   }
}
