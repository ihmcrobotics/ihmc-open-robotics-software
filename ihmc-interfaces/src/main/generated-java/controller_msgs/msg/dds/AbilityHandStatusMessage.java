package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandStatusMessage extends Packet<AbilityHandStatusMessage> implements Settable<AbilityHandStatusMessage>, EpsilonComparable<AbilityHandStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte robot_side_ = (byte) 255;
   public double[] finger_positions_;

   public AbilityHandStatusMessage()
   {
      finger_positions_ = new double[6];

   }

   public AbilityHandStatusMessage(AbilityHandStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandStatusMessage other)
   {
      robot_side_ = other.robot_side_;

      for(int i1 = 0; i1 < finger_positions_.length; ++i1)
      {
            finger_positions_[i1] = other.finger_positions_[i1];

      }

   }

   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   public double[] getFingerPositions()
   {
      return finger_positions_;
   }


   public static Supplier<AbilityHandStatusMessagePubSubType> getPubSubType()
   {
      return AbilityHandStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      for(int i3 = 0; i3 < finger_positions_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.finger_positions_[i3], other.finger_positions_[i3], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandStatusMessage)) return false;

      AbilityHandStatusMessage otherMyClass = (AbilityHandStatusMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      for(int i5 = 0; i5 < finger_positions_.length; ++i5)
      {
                if(this.finger_positions_[i5] != otherMyClass.finger_positions_[i5]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandStatusMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("finger_positions=");
      builder.append(java.util.Arrays.toString(this.finger_positions_));
      builder.append("}");
      return builder.toString();
   }
}
