package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandDesiredCommandMessage extends Packet<AbilityHandDesiredCommandMessage> implements Settable<AbilityHandDesiredCommandMessage>, EpsilonComparable<AbilityHandDesiredCommandMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte robot_side_ = (byte) 255;
   public double[] desired_finger_positions_;

   public AbilityHandDesiredCommandMessage()
   {
      desired_finger_positions_ = new double[6];

   }

   public AbilityHandDesiredCommandMessage(AbilityHandDesiredCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandDesiredCommandMessage other)
   {
      robot_side_ = other.robot_side_;

      for(int i1 = 0; i1 < desired_finger_positions_.length; ++i1)
      {
            desired_finger_positions_[i1] = other.desired_finger_positions_[i1];

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


   public double[] getDesiredFingerPositions()
   {
      return desired_finger_positions_;
   }


   public static Supplier<AbilityHandDesiredCommandMessagePubSubType> getPubSubType()
   {
      return AbilityHandDesiredCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandDesiredCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandDesiredCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      for(int i3 = 0; i3 < desired_finger_positions_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_finger_positions_[i3], other.desired_finger_positions_[i3], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandDesiredCommandMessage)) return false;

      AbilityHandDesiredCommandMessage otherMyClass = (AbilityHandDesiredCommandMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      for(int i5 = 0; i5 < desired_finger_positions_.length; ++i5)
      {
                if(this.desired_finger_positions_[i5] != otherMyClass.desired_finger_positions_[i5]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandDesiredCommandMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("desired_finger_positions=");
      builder.append(java.util.Arrays.toString(this.desired_finger_positions_));
      builder.append("}");
      return builder.toString();
   }
}
