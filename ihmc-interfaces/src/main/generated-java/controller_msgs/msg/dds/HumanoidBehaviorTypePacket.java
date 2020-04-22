package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class HumanoidBehaviorTypePacket extends Packet<HumanoidBehaviorTypePacket> implements Settable<HumanoidBehaviorTypePacket>, EpsilonComparable<HumanoidBehaviorTypePacket>
{

   public static final byte STOP = (byte) 0;

   public static final byte TEST = (byte) 1;

   public static final byte WALK_TO_LOCATION = (byte) 2;

   public static final byte WALK_TO_GOAL = (byte) 3;

   public static final byte DIAGNOSTIC = (byte) 4;

   public static final byte PICK_UP_BALL = (byte) 5;

   public static final byte RESET_ROBOT = (byte) 6;

   public static final byte TURN_VALVE = (byte) 7;

   public static final byte WALK_THROUGH_DOOR = (byte) 8;

   public static final byte EXAMPLE_BEHAVIOR = (byte) 9;

   public static final byte BALL_DETECTION = (byte) 10;

   public static final byte TEST_PIPELINE = (byte) 11;

   public static final byte TEST_STATEMACHINE = (byte) 12;

   public static final byte FOLLOW_FIDUCIAL_50 = (byte) 13;

   public static final byte LOCATE_FIDUCIAL = (byte) 14;

   public static final byte WALK_OVER_TERRAIN = (byte) 15;

   public static final byte FOLLOW_VALVE = (byte) 16;

   public static final byte LOCATE_VALVE = (byte) 17;

   public static final byte WALK_OVER_TERRAIN_TO_VALVE = (byte) 18;

   public static final byte DEBUG_PARTIAL_FOOTHOLDS = (byte) 19;

   public static final byte WALK_TO_GOAL_ANYTIME_PLANNER = (byte) 20;

   public static final byte TEST_ICP_OPTIMIZATION = (byte) 21;

   public static final byte TEST_GC_GENERATION = (byte) 22;

   public static final byte TEST_SMOOTH_ICP_PLANNER = (byte) 23;

   public static final byte PUSH_AND_WALK = (byte) 24;

   public static final byte COLLABORATIVE_TASK = (byte) 25;

   public static final byte FIRE_FIGHTING = (byte) 26;

   public static final byte CUTTING_WALL = (byte) 27;

   public static final byte REPEATEDLY_WALK_FOOTSTEP_LIST = (byte) 28;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte humanoid_behavior_type_ = (byte) 255;

   public HumanoidBehaviorTypePacket()
   {



   }

   public HumanoidBehaviorTypePacket(HumanoidBehaviorTypePacket other)
   {
      this();
      set(other);
   }

   public void set(HumanoidBehaviorTypePacket other)
   {

      sequence_id_ = other.sequence_id_;


      humanoid_behavior_type_ = other.humanoid_behavior_type_;

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


   public void setHumanoidBehaviorType(byte humanoid_behavior_type)
   {
      humanoid_behavior_type_ = humanoid_behavior_type;
   }
   public byte getHumanoidBehaviorType()
   {
      return humanoid_behavior_type_;
   }


   public static Supplier<HumanoidBehaviorTypePacketPubSubType> getPubSubType()
   {
      return HumanoidBehaviorTypePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HumanoidBehaviorTypePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorTypePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.humanoid_behavior_type_, other.humanoid_behavior_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HumanoidBehaviorTypePacket)) return false;

      HumanoidBehaviorTypePacket otherMyClass = (HumanoidBehaviorTypePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.humanoid_behavior_type_ != otherMyClass.humanoid_behavior_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HumanoidBehaviorTypePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("humanoid_behavior_type=");
      builder.append(this.humanoid_behavior_type_);
      builder.append("}");
      return builder.toString();
   }
}
