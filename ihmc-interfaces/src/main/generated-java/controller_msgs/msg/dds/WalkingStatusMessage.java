package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. The controller sends this message to
 * notify the user of the status of walking.
 */
public class WalkingStatusMessage extends Packet<WalkingStatusMessage> implements Settable<WalkingStatusMessage>, EpsilonComparable<WalkingStatusMessage>
{
   public static final byte STARTED = (byte) 0;
   public static final byte COMPLETED = (byte) 1;
   public static final byte ABORT_REQUESTED = (byte) 2;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * Status of walking.
    */
   public byte walking_status_ = (byte) 255;

   public WalkingStatusMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public WalkingStatusMessage(WalkingStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkingStatusMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      walking_status_ = other.walking_status_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * Status of walking.
    */
   public void setWalkingStatus(byte walking_status)
   {
      walking_status_ = walking_status;
   }

   /**
    * Status of walking.
    */
   public byte getWalkingStatus()
   {
      return walking_status_;
   }

   @Override
   public boolean epsilonEquals(WalkingStatusMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.walking_status_, other.walking_status_, epsilon))
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
      if (!(other instanceof WalkingStatusMessage))
         return false;

      WalkingStatusMessage otherMyClass = (WalkingStatusMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.walking_status_ != otherMyClass.walking_status_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkingStatusMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("walking_status=");
      builder.append(this.walking_status_);
      builder.append("}");
      return builder.toString();
   }
}
