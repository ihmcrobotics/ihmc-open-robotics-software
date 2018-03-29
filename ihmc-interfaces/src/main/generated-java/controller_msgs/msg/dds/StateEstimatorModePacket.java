package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API.
 */
public class StateEstimatorModePacket extends Packet<StateEstimatorModePacket>
      implements Settable<StateEstimatorModePacket>, EpsilonComparable<StateEstimatorModePacket>
{
   public static final byte NORMAL = (byte) 0;
   public static final byte FROZEN = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte requested_state_estimator_mode_ = (byte) 255;

   public StateEstimatorModePacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public StateEstimatorModePacket(StateEstimatorModePacket other)
   {
      this();
      set(other);
   }

   public void set(StateEstimatorModePacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      requested_state_estimator_mode_ = other.requested_state_estimator_mode_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setRequestedStateEstimatorMode(byte requested_state_estimator_mode)
   {
      requested_state_estimator_mode_ = requested_state_estimator_mode;
   }

   public byte getRequestedStateEstimatorMode()
   {
      return requested_state_estimator_mode_;
   }

   @Override
   public boolean epsilonEquals(StateEstimatorModePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_state_estimator_mode_, other.requested_state_estimator_mode_, epsilon))
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
      if (!(other instanceof StateEstimatorModePacket))
         return false;

      StateEstimatorModePacket otherMyClass = (StateEstimatorModePacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.requested_state_estimator_mode_ != otherMyClass.requested_state_estimator_mode_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StateEstimatorModePacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("requested_state_estimator_mode=");
      builder.append(this.requested_state_estimator_mode_);
      builder.append("}");
      return builder.toString();
   }
}
