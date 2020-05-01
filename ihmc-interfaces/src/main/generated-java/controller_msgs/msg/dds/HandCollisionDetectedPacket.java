package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message for the IHMC humanoid behavior module.
       */
public class HandCollisionDetectedPacket extends Packet<HandCollisionDetectedPacket> implements Settable<HandCollisionDetectedPacket>, EpsilonComparable<HandCollisionDetectedPacket>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte robot_side_ = (byte) 255;

   public int collision_severity_level_one_to_three_;

   public HandCollisionDetectedPacket()
   {




   }

   public HandCollisionDetectedPacket(HandCollisionDetectedPacket other)
   {
      this();
      set(other);
   }

   public void set(HandCollisionDetectedPacket other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;


      collision_severity_level_one_to_three_ = other.collision_severity_level_one_to_three_;

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


   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   public byte getRobotSide()
   {
      return robot_side_;
   }


   public void setCollisionSeverityLevelOneToThree(int collision_severity_level_one_to_three)
   {
      collision_severity_level_one_to_three_ = collision_severity_level_one_to_three;
   }
   public int getCollisionSeverityLevelOneToThree()
   {
      return collision_severity_level_one_to_three_;
   }


   public static Supplier<HandCollisionDetectedPacketPubSubType> getPubSubType()
   {
      return HandCollisionDetectedPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandCollisionDetectedPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandCollisionDetectedPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_severity_level_one_to_three_, other.collision_severity_level_one_to_three_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandCollisionDetectedPacket)) return false;

      HandCollisionDetectedPacket otherMyClass = (HandCollisionDetectedPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if(this.collision_severity_level_one_to_three_ != otherMyClass.collision_severity_level_one_to_three_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandCollisionDetectedPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("collision_severity_level_one_to_three=");
      builder.append(this.collision_severity_level_one_to_three_);
      builder.append("}");
      return builder.toString();
   }
}
