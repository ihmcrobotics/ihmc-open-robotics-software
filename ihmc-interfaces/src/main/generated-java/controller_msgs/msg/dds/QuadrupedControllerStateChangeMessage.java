package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message notifies the user of a change in the high level controller state.
       * This message's primary use is to signal a requested state change is completed.
       */
public class QuadrupedControllerStateChangeMessage extends Packet<QuadrupedControllerStateChangeMessage> implements Settable<QuadrupedControllerStateChangeMessage>, EpsilonComparable<QuadrupedControllerStateChangeMessage>
{
   public static final byte JOINT_INITIALIZATION = (byte) 0;
   public static final byte DO_NOTHING = (byte) 1;
   public static final byte STAND_PREP = (byte) 2;
   public static final byte STAND_READY = (byte) 3;
   public static final byte FREEZE = (byte) 4;
   public static final byte STEPPING = (byte) 5;
   public static final byte FALL = (byte) 6;
   /**
            * Specifies the controller's state prior to transition.
            */
   public byte initial_controller_name_ = (byte) 255;
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte end_controller_name_ = (byte) 255;

   public QuadrupedControllerStateChangeMessage()
   {
   }

   public QuadrupedControllerStateChangeMessage(QuadrupedControllerStateChangeMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedControllerStateChangeMessage other)
   {
      initial_controller_name_ = other.initial_controller_name_;

      end_controller_name_ = other.end_controller_name_;

   }

   /**
            * Specifies the controller's state prior to transition.
            */
   public void setInitialControllerName(byte initial_controller_name)
   {
      initial_controller_name_ = initial_controller_name;
   }
   /**
            * Specifies the controller's state prior to transition.
            */
   public byte getInitialControllerName()
   {
      return initial_controller_name_;
   }

   /**
            * Specifies the state the controller has transitioned into.
            */
   public void setEndControllerName(byte end_controller_name)
   {
      end_controller_name_ = end_controller_name;
   }
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte getEndControllerName()
   {
      return end_controller_name_;
   }


   @Override
   public boolean epsilonEquals(QuadrupedControllerStateChangeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_controller_name_, other.initial_controller_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_controller_name_, other.end_controller_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedControllerStateChangeMessage)) return false;

      QuadrupedControllerStateChangeMessage otherMyClass = (QuadrupedControllerStateChangeMessage) other;

      if(this.initial_controller_name_ != otherMyClass.initial_controller_name_) return false;

      if(this.end_controller_name_ != otherMyClass.end_controller_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedControllerStateChangeMessage {");
      builder.append("initial_controller_name=");
      builder.append(this.initial_controller_name_);      builder.append(", ");
      builder.append("end_controller_name=");
      builder.append(this.end_controller_name_);
      builder.append("}");
      return builder.toString();
   }
}
