package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message is used to switch the control scheme between different control mode.
       */
public class QuadrupedRequestedControllerStateMessage extends Packet<QuadrupedRequestedControllerStateMessage> implements Settable<QuadrupedRequestedControllerStateMessage>, EpsilonComparable<QuadrupedRequestedControllerStateMessage>
{
   public static final byte REQUEST_DO_NOTHING = (byte) 0;
   public static final byte REQUEST_STAND_PREP = (byte) 1;
   public static final byte REQUEST_FREEZE = (byte) 2;
   public static final byte REQUEST_STEPPING = (byte) 3;
   public static final byte REQUEST_FALL = (byte) 4;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the which state the controller should transition into.
            */
   public byte quadruped_controller_name_ = (byte) 255;

   public QuadrupedRequestedControllerStateMessage()
   {
   }

   public QuadrupedRequestedControllerStateMessage(QuadrupedRequestedControllerStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedRequestedControllerStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      quadruped_controller_name_ = other.quadruped_controller_name_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Specifies the which state the controller should transition into.
            */
   public void setQuadrupedControllerName(byte quadruped_controller_name)
   {
      quadruped_controller_name_ = quadruped_controller_name;
   }
   /**
            * Specifies the which state the controller should transition into.
            */
   public byte getQuadrupedControllerName()
   {
      return quadruped_controller_name_;
   }


   public static Supplier<QuadrupedRequestedControllerStateMessagePubSubType> getPubSubType()
   {
      return QuadrupedRequestedControllerStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedRequestedControllerStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quadruped_controller_name_, other.quadruped_controller_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedRequestedControllerStateMessage)) return false;

      QuadrupedRequestedControllerStateMessage otherMyClass = (QuadrupedRequestedControllerStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.quadruped_controller_name_ != otherMyClass.quadruped_controller_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedRequestedControllerStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("quadruped_controller_name=");
      builder.append(this.quadruped_controller_name_);
      builder.append("}");
      return builder.toString();
   }
}
