package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message is used to switch the
 * control scheme between different control mode.
 */
public class HighLevelStateMessage extends Packet<HighLevelStateMessage> implements Settable<HighLevelStateMessage>, EpsilonComparable<HighLevelStateMessage>
{
   public static final byte DO_NOTHING_BEHAVIOR = (byte) 0;
   public static final byte STAND_PREP_STATE = (byte) 1;
   public static final byte STAND_READY = (byte) 2;
   public static final byte FREEZE_STATE = (byte) 3;
   public static final byte STAND_TRANSITION_STATE = (byte) 4;
   public static final byte WALKING = (byte) 5;
   public static final byte DIAGNOSTICS = (byte) 6;
   public static final byte CALIBRATION = (byte) 7;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * Specifies the which state the controller should transition into.
    */
   public byte high_level_controller_name_ = (byte) 255;

   public HighLevelStateMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public HighLevelStateMessage(HighLevelStateMessage other)
   {
      this();
      set(other);
   }

   public void set(HighLevelStateMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      high_level_controller_name_ = other.high_level_controller_name_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * Specifies the which state the controller should transition into.
    */
   public void setHighLevelControllerName(byte high_level_controller_name)
   {
      high_level_controller_name_ = high_level_controller_name;
   }

   /**
    * Specifies the which state the controller should transition into.
    */
   public byte getHighLevelControllerName()
   {
      return high_level_controller_name_;
   }

   @Override
   public boolean epsilonEquals(HighLevelStateMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.high_level_controller_name_, other.high_level_controller_name_, epsilon))
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
      if (!(other instanceof HighLevelStateMessage))
         return false;

      HighLevelStateMessage otherMyClass = (HighLevelStateMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.high_level_controller_name_ != otherMyClass.high_level_controller_name_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HighLevelStateMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("high_level_controller_name=");
      builder.append(this.high_level_controller_name_);
      builder.append("}");
      return builder.toString();
   }
}
