package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Atlas specific message.
 */
public class AtlasLowLevelControlModeMessage extends Packet<AtlasLowLevelControlModeMessage>
      implements Settable<AtlasLowLevelControlModeMessage>, EpsilonComparable<AtlasLowLevelControlModeMessage>
{
   public static final byte ATLAS_LOW_LEVEL_CONTROL_MODE_STAND_PREP = (byte) 0;
   public static final byte ATLAS_LOW_LEVEL_CONTROL_MODE_FREEZE = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte requested_atlas_low_level_control_mode_ = (byte) 255;

   public AtlasLowLevelControlModeMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public AtlasLowLevelControlModeMessage(AtlasLowLevelControlModeMessage other)
   {
      this();
      set(other);
   }

   public void set(AtlasLowLevelControlModeMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      requested_atlas_low_level_control_mode_ = other.requested_atlas_low_level_control_mode_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setRequestedAtlasLowLevelControlMode(byte requested_atlas_low_level_control_mode)
   {
      requested_atlas_low_level_control_mode_ = requested_atlas_low_level_control_mode;
   }

   public byte getRequestedAtlasLowLevelControlMode()
   {
      return requested_atlas_low_level_control_mode_;
   }

   @Override
   public boolean epsilonEquals(AtlasLowLevelControlModeMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_atlas_low_level_control_mode_, other.requested_atlas_low_level_control_mode_, epsilon))
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
      if (!(other instanceof AtlasLowLevelControlModeMessage))
         return false;

      AtlasLowLevelControlModeMessage otherMyClass = (AtlasLowLevelControlModeMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.requested_atlas_low_level_control_mode_ != otherMyClass.requested_atlas_low_level_control_mode_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasLowLevelControlModeMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("requested_atlas_low_level_control_mode=");
      builder.append(this.requested_atlas_low_level_control_mode_);
      builder.append("}");
      return builder.toString();
   }
}
