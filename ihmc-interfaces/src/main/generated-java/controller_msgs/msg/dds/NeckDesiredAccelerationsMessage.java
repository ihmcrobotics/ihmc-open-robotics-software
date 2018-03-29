package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message gives the user the option to bypass IHMC feedback controllers for the neck joints by sending desired neck joint accelerations.
 * One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations.
 */
public class NeckDesiredAccelerationsMessage extends Packet<NeckDesiredAccelerationsMessage>
      implements Settable<NeckDesiredAccelerationsMessage>, EpsilonComparable<NeckDesiredAccelerationsMessage>
{
   /**
    * The desired joint acceleration information.
    */
   public controller_msgs.msg.dds.DesiredAccelerationsMessage desired_accelerations_;

   public NeckDesiredAccelerationsMessage()
   {
      desired_accelerations_ = new controller_msgs.msg.dds.DesiredAccelerationsMessage();
   }

   public NeckDesiredAccelerationsMessage(NeckDesiredAccelerationsMessage other)
   {
      set(other);
   }

   public void set(NeckDesiredAccelerationsMessage other)
   {
      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.staticCopy(other.desired_accelerations_, desired_accelerations_);
   }

   /**
    * The desired joint acceleration information.
    */
   public controller_msgs.msg.dds.DesiredAccelerationsMessage getDesiredAccelerations()
   {
      return desired_accelerations_;
   }

   @Override
   public boolean epsilonEquals(NeckDesiredAccelerationsMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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
      if (!(other instanceof NeckDesiredAccelerationsMessage))
         return false;

      NeckDesiredAccelerationsMessage otherMyClass = (NeckDesiredAccelerationsMessage) other;

      if (!this.desired_accelerations_.equals(otherMyClass.desired_accelerations_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NeckDesiredAccelerationsMessage {");
      builder.append("desired_accelerations=");
      builder.append(this.desired_accelerations_);

      builder.append("}");
      return builder.toString();
   }
}
