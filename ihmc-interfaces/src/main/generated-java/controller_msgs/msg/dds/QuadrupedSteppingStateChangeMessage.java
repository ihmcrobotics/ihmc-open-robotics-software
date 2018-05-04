package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message notifies the user of a change in the stepping controller state.
       * This message's primary use is to signal a requested state change is completed.
       */
public class QuadrupedSteppingStateChangeMessage extends Packet<QuadrupedSteppingStateChangeMessage> implements Settable<QuadrupedSteppingStateChangeMessage>, EpsilonComparable<QuadrupedSteppingStateChangeMessage>
{
   public static final byte STAND = (byte) 0;
   public static final byte STEP = (byte) 1;
   public static final byte SOLE_WAYPOINT = (byte) 2;
   /**
            * Specifies the controller's state prior to transition.
            */
   public byte initial_stepping_controller_name_ = (byte) 255;
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte end_stepping_controller_name_ = (byte) 255;

   public QuadrupedSteppingStateChangeMessage()
   {
   }

   public QuadrupedSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedSteppingStateChangeMessage other)
   {
      initial_stepping_controller_name_ = other.initial_stepping_controller_name_;

      end_stepping_controller_name_ = other.end_stepping_controller_name_;

   }

   /**
            * Specifies the controller's state prior to transition.
            */
   public void setInitialSteppingControllerName(byte initial_stepping_controller_name)
   {
      initial_stepping_controller_name_ = initial_stepping_controller_name;
   }
   /**
            * Specifies the controller's state prior to transition.
            */
   public byte getInitialSteppingControllerName()
   {
      return initial_stepping_controller_name_;
   }

   /**
            * Specifies the state the controller has transitioned into.
            */
   public void setEndSteppingControllerName(byte end_stepping_controller_name)
   {
      end_stepping_controller_name_ = end_stepping_controller_name;
   }
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte getEndSteppingControllerName()
   {
      return end_stepping_controller_name_;
   }


   public static Supplier<QuadrupedSteppingStateChangeMessagePubSubType> getPubSubType()
   {
      return QuadrupedSteppingStateChangeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedSteppingStateChangeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedSteppingStateChangeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_stepping_controller_name_, other.initial_stepping_controller_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_stepping_controller_name_, other.end_stepping_controller_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedSteppingStateChangeMessage)) return false;

      QuadrupedSteppingStateChangeMessage otherMyClass = (QuadrupedSteppingStateChangeMessage) other;

      if(this.initial_stepping_controller_name_ != otherMyClass.initial_stepping_controller_name_) return false;

      if(this.end_stepping_controller_name_ != otherMyClass.end_stepping_controller_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedSteppingStateChangeMessage {");
      builder.append("initial_stepping_controller_name=");
      builder.append(this.initial_stepping_controller_name_);      builder.append(", ");
      builder.append("end_stepping_controller_name=");
      builder.append(this.end_stepping_controller_name_);
      builder.append("}");
      return builder.toString();
   }
}
