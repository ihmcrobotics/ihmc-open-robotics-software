package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

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
   public byte initial_quadruped_controller_enum_ = (byte) 255;
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte end_quadruped_controller_enum_ = (byte) 255;

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
      initial_quadruped_controller_enum_ = other.initial_quadruped_controller_enum_;

      end_quadruped_controller_enum_ = other.end_quadruped_controller_enum_;

   }

   /**
            * Specifies the controller's state prior to transition.
            */
   public void setInitialQuadrupedControllerEnum(byte initial_quadruped_controller_enum)
   {
      initial_quadruped_controller_enum_ = initial_quadruped_controller_enum;
   }
   /**
            * Specifies the controller's state prior to transition.
            */
   public byte getInitialQuadrupedControllerEnum()
   {
      return initial_quadruped_controller_enum_;
   }

   /**
            * Specifies the state the controller has transitioned into.
            */
   public void setEndQuadrupedControllerEnum(byte end_quadruped_controller_enum)
   {
      end_quadruped_controller_enum_ = end_quadruped_controller_enum;
   }
   /**
            * Specifies the state the controller has transitioned into.
            */
   public byte getEndQuadrupedControllerEnum()
   {
      return end_quadruped_controller_enum_;
   }


   public static Supplier<QuadrupedControllerStateChangeMessagePubSubType> getPubSubType()
   {
      return QuadrupedControllerStateChangeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedControllerStateChangeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedControllerStateChangeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_quadruped_controller_enum_, other.initial_quadruped_controller_enum_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_quadruped_controller_enum_, other.end_quadruped_controller_enum_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedControllerStateChangeMessage)) return false;

      QuadrupedControllerStateChangeMessage otherMyClass = (QuadrupedControllerStateChangeMessage) other;

      if(this.initial_quadruped_controller_enum_ != otherMyClass.initial_quadruped_controller_enum_) return false;

      if(this.end_quadruped_controller_enum_ != otherMyClass.end_quadruped_controller_enum_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedControllerStateChangeMessage {");
      builder.append("initial_quadruped_controller_enum=");
      builder.append(this.initial_quadruped_controller_enum_);      builder.append(", ");
      builder.append("end_quadruped_controller_enum=");
      builder.append(this.end_quadruped_controller_enum_);
      builder.append("}");
      return builder.toString();
   }
}
