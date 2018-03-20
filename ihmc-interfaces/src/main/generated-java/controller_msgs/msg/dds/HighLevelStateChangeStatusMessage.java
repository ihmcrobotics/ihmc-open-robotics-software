package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message notifies the user of a change in the high level state.
 * This message's primary use is to signal a requested state change is completed.
 */
public class HighLevelStateChangeStatusMessage extends Packet<HighLevelStateChangeStatusMessage>
      implements Settable<HighLevelStateChangeStatusMessage>, EpsilonComparable<HighLevelStateChangeStatusMessage>
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
    * Specifies the controller's state prior to transition.
    */
   public byte initial_high_level_controller_name_ = (byte) 255;
   /**
    * Specifies the state the controller has transitioned into.
    */
   public byte end_high_level_controller_name_ = (byte) 255;

   public HighLevelStateChangeStatusMessage()
   {

   }

   public HighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage other)
   {
      set(other);
   }

   public void set(HighLevelStateChangeStatusMessage other)
   {
      initial_high_level_controller_name_ = other.initial_high_level_controller_name_;

      end_high_level_controller_name_ = other.end_high_level_controller_name_;
   }

   /**
    * Specifies the controller's state prior to transition.
    */
   public byte getInitialHighLevelControllerName()
   {
      return initial_high_level_controller_name_;
   }

   /**
    * Specifies the controller's state prior to transition.
    */
   public void setInitialHighLevelControllerName(byte initial_high_level_controller_name)
   {
      initial_high_level_controller_name_ = initial_high_level_controller_name;
   }

   /**
    * Specifies the state the controller has transitioned into.
    */
   public byte getEndHighLevelControllerName()
   {
      return end_high_level_controller_name_;
   }

   /**
    * Specifies the state the controller has transitioned into.
    */
   public void setEndHighLevelControllerName(byte end_high_level_controller_name)
   {
      end_high_level_controller_name_ = end_high_level_controller_name;
   }

   @Override
   public boolean epsilonEquals(HighLevelStateChangeStatusMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_high_level_controller_name_, other.initial_high_level_controller_name_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_high_level_controller_name_, other.end_high_level_controller_name_, epsilon))
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
      if (!(other instanceof HighLevelStateChangeStatusMessage))
         return false;

      HighLevelStateChangeStatusMessage otherMyClass = (HighLevelStateChangeStatusMessage) other;

      if (this.initial_high_level_controller_name_ != otherMyClass.initial_high_level_controller_name_)
         return false;

      if (this.end_high_level_controller_name_ != otherMyClass.end_high_level_controller_name_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HighLevelStateChangeStatusMessage {");
      builder.append("initial_high_level_controller_name=");
      builder.append(this.initial_high_level_controller_name_);

      builder.append(", ");
      builder.append("end_high_level_controller_name=");
      builder.append(this.end_high_level_controller_name_);

      builder.append("}");
      return builder.toString();
   }
}