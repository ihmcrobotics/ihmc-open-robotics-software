package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Atlas specific message.
 */
public class AtlasLowLevelControlModeMessage extends Packet<AtlasLowLevelControlModeMessage>
      implements Settable<AtlasLowLevelControlModeMessage>, EpsilonComparable<AtlasLowLevelControlModeMessage>
{
   public static final byte ATLAS_LOW_LEVEL_CONTROL_MODE_STAND_PREP = (byte) 0;
   public static final byte ATLAS_LOW_LEVEL_CONTROL_MODE_FREEZE = (byte) 1;
   public byte requested_atlas_low_level_control_mode_ = (byte) 255;

   public AtlasLowLevelControlModeMessage()
   {
   }

   public AtlasLowLevelControlModeMessage(AtlasLowLevelControlModeMessage other)
   {
      set(other);
   }

   public void set(AtlasLowLevelControlModeMessage other)
   {
      requested_atlas_low_level_control_mode_ = other.requested_atlas_low_level_control_mode_;
   }

   public byte getRequestedAtlasLowLevelControlMode()
   {
      return requested_atlas_low_level_control_mode_;
   }

   public void setRequestedAtlasLowLevelControlMode(byte requested_atlas_low_level_control_mode)
   {
      requested_atlas_low_level_control_mode_ = requested_atlas_low_level_control_mode;
   }

   @Override
   public boolean epsilonEquals(AtlasLowLevelControlModeMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      if (this.requested_atlas_low_level_control_mode_ != otherMyClass.requested_atlas_low_level_control_mode_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasLowLevelControlModeMessage {");
      builder.append("requested_atlas_low_level_control_mode=");
      builder.append(this.requested_atlas_low_level_control_mode_);

      builder.append("}");
      return builder.toString();
   }
}