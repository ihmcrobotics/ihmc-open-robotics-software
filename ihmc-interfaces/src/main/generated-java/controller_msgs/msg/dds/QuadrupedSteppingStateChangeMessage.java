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
   public byte initial_quadruped_stepping_state_enum_ = (byte) 255;

   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte end_quadruped_stepping_state_enum_ = (byte) 255;

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

      initial_quadruped_stepping_state_enum_ = other.initial_quadruped_stepping_state_enum_;


      end_quadruped_stepping_state_enum_ = other.end_quadruped_stepping_state_enum_;

   }


   /**
            * Specifies the controller's state prior to transition.
            */
   public void setInitialQuadrupedSteppingStateEnum(byte initial_quadruped_stepping_state_enum)
   {
      initial_quadruped_stepping_state_enum_ = initial_quadruped_stepping_state_enum;
   }
   /**
            * Specifies the controller's state prior to transition.
            */
   public byte getInitialQuadrupedSteppingStateEnum()
   {
      return initial_quadruped_stepping_state_enum_;
   }


   /**
            * Specifies the state the controller has transitioned into.
            */
   public void setEndQuadrupedSteppingStateEnum(byte end_quadruped_stepping_state_enum)
   {
      end_quadruped_stepping_state_enum_ = end_quadruped_stepping_state_enum;
   }
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte getEndQuadrupedSteppingStateEnum()
   {
      return end_quadruped_stepping_state_enum_;
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


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_quadruped_stepping_state_enum_, other.initial_quadruped_stepping_state_enum_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_quadruped_stepping_state_enum_, other.end_quadruped_stepping_state_enum_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedSteppingStateChangeMessage)) return false;

      QuadrupedSteppingStateChangeMessage otherMyClass = (QuadrupedSteppingStateChangeMessage) other;


      if(this.initial_quadruped_stepping_state_enum_ != otherMyClass.initial_quadruped_stepping_state_enum_) return false;


      if(this.end_quadruped_stepping_state_enum_ != otherMyClass.end_quadruped_stepping_state_enum_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedSteppingStateChangeMessage {");

      builder.append("initial_quadruped_stepping_state_enum=");
      builder.append(this.initial_quadruped_stepping_state_enum_);      builder.append(", ");

      builder.append("end_quadruped_stepping_state_enum=");
      builder.append(this.end_quadruped_stepping_state_enum_);
      builder.append("}");
      return builder.toString();
   }
}
