package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is sent to command the robot to move as if controlled by a joystick.
       * The forward, right and clockwise parameters are a direction of travel and speed
       * represented as a set of 3 movement impulses:
       * - forwards/backwards
       * - left/right
       * - clockwise/counter-clockwise rotation
       * 
       * Each impulse value is a floating point number, v, with -1 <= v <= 1, where
       * v<0 indicates backwards/left/counter-clockwise movement at (-v)*max speed,
       * v=0 indicates no change,
       * v>0 indicates forwards/right/clockwise at v*max speed.
       */
public class DirectionalControlInputMessage extends Packet<DirectionalControlInputMessage> implements Settable<DirectionalControlInputMessage>, EpsilonComparable<DirectionalControlInputMessage>
{
   public long sequence_id_;
   public double forward_;
   public double right_;
   public double clockwise_;

   public DirectionalControlInputMessage()
   {
   }

   public DirectionalControlInputMessage(DirectionalControlInputMessage other)
   {
      this();
      set(other);
   }

   public void set(DirectionalControlInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      forward_ = other.forward_;

      right_ = other.right_;

      clockwise_ = other.clockwise_;

   }

   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setForward(double forward)
   {
      forward_ = forward;
   }
   public double getForward()
   {
      return forward_;
   }

   public void setRight(double right)
   {
      right_ = right;
   }
   public double getRight()
   {
      return right_;
   }

   public void setClockwise(double clockwise)
   {
      clockwise_ = clockwise;
   }
   public double getClockwise()
   {
      return clockwise_;
   }


   public static Supplier<DirectionalControlInputMessagePubSubType> getPubSubType()
   {
      return DirectionalControlInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DirectionalControlInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DirectionalControlInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_, other.forward_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_, other.right_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.clockwise_, other.clockwise_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DirectionalControlInputMessage)) return false;

      DirectionalControlInputMessage otherMyClass = (DirectionalControlInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.forward_ != otherMyClass.forward_) return false;

      if(this.right_ != otherMyClass.right_) return false;

      if(this.clockwise_ != otherMyClass.clockwise_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DirectionalControlInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("forward=");
      builder.append(this.forward_);      builder.append(", ");
      builder.append("right=");
      builder.append(this.right_);      builder.append(", ");
      builder.append("clockwise=");
      builder.append(this.clockwise_);
      builder.append("}");
      return builder.toString();
   }
}
