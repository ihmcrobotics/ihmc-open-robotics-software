package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandCommandMessage extends Packet<AbilityHandCommandMessage> implements Settable<AbilityHandCommandMessage>, EpsilonComparable<AbilityHandCommandMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte robot_side_ = (byte) 255;
   public float[] finger_positions_degrees_;

   public AbilityHandCommandMessage()
   {
      finger_positions_degrees_ = new float[6];

   }

   public AbilityHandCommandMessage(AbilityHandCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandCommandMessage other)
   {
      robot_side_ = other.robot_side_;

      for(int i1 = 0; i1 < finger_positions_degrees_.length; ++i1)
      {
            finger_positions_degrees_[i1] = other.finger_positions_degrees_[i1];

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


   public float[] getFingerPositionsDegrees()
   {
      return finger_positions_degrees_;
   }


   public static Supplier<AbilityHandCommandMessagePubSubType> getPubSubType()
   {
      return AbilityHandCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      for(int i3 = 0; i3 < finger_positions_degrees_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.finger_positions_degrees_[i3], other.finger_positions_degrees_[i3], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandCommandMessage)) return false;

      AbilityHandCommandMessage otherMyClass = (AbilityHandCommandMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      for(int i5 = 0; i5 < finger_positions_degrees_.length; ++i5)
      {
                if(this.finger_positions_degrees_[i5] != otherMyClass.finger_positions_degrees_[i5]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandCommandMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("finger_positions_degrees=");
      builder.append(java.util.Arrays.toString(this.finger_positions_degrees_));
      builder.append("}");
      return builder.toString();
   }
}
