package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used to configure the exploration for a RRT-based planner.
       * Main usage is the IHMC WholeBodyTrajectoryToolbox.
       */
public class RigidBodyExplorationConfigurationMessage extends Packet<RigidBodyExplorationConfigurationMessage> implements Settable<RigidBodyExplorationConfigurationMessage>, EpsilonComparable<RigidBodyExplorationConfigurationMessage>
{

   public static final byte CONFIGURATION_SPACE_NAME_X = (byte) 0;

   public static final byte CONFIGURATION_SPACE_NAME_Y = (byte) 1;

   public static final byte CONFIGURATION_SPACE_NAME_Z = (byte) 2;

   public static final byte CONFIGURATION_SPACE_NAME_ROLL = (byte) 3;

   public static final byte CONFIGURATION_SPACE_NAME_PITCH = (byte) 4;

   public static final byte CONFIGURATION_SPACE_NAME_YAW = (byte) 5;

   public static final byte CONFIGURATION_SPACE_NAME_SO3 = (byte) 6;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public int rigid_body_hash_code_;

   public us.ihmc.idl.IDLSequence.Byte  configuration_space_names_to_explore_;

   public us.ihmc.idl.IDLSequence.Double  exploration_range_upper_limits_;

   public us.ihmc.idl.IDLSequence.Double  exploration_range_lower_limits_;

   public RigidBodyExplorationConfigurationMessage()
   {



      configuration_space_names_to_explore_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");


      exploration_range_upper_limits_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


      exploration_range_lower_limits_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


   }

   public RigidBodyExplorationConfigurationMessage(RigidBodyExplorationConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(RigidBodyExplorationConfigurationMessage other)
   {

      sequence_id_ = other.sequence_id_;


      rigid_body_hash_code_ = other.rigid_body_hash_code_;


      configuration_space_names_to_explore_.set(other.configuration_space_names_to_explore_);

      exploration_range_upper_limits_.set(other.exploration_range_upper_limits_);

      exploration_range_lower_limits_.set(other.exploration_range_lower_limits_);
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


   public void setRigidBodyHashCode(int rigid_body_hash_code)
   {
      rigid_body_hash_code_ = rigid_body_hash_code;
   }
   public int getRigidBodyHashCode()
   {
      return rigid_body_hash_code_;
   }



   public us.ihmc.idl.IDLSequence.Byte  getConfigurationSpaceNamesToExplore()
   {
      return configuration_space_names_to_explore_;
   }



   public us.ihmc.idl.IDLSequence.Double  getExplorationRangeUpperLimits()
   {
      return exploration_range_upper_limits_;
   }



   public us.ihmc.idl.IDLSequence.Double  getExplorationRangeLowerLimits()
   {
      return exploration_range_lower_limits_;
   }


   public static Supplier<RigidBodyExplorationConfigurationMessagePubSubType> getPubSubType()
   {
      return RigidBodyExplorationConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RigidBodyExplorationConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RigidBodyExplorationConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rigid_body_hash_code_, other.rigid_body_hash_code_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.configuration_space_names_to_explore_, other.configuration_space_names_to_explore_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.exploration_range_upper_limits_, other.exploration_range_upper_limits_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.exploration_range_lower_limits_, other.exploration_range_lower_limits_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RigidBodyExplorationConfigurationMessage)) return false;

      RigidBodyExplorationConfigurationMessage otherMyClass = (RigidBodyExplorationConfigurationMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.rigid_body_hash_code_ != otherMyClass.rigid_body_hash_code_) return false;


      if (!this.configuration_space_names_to_explore_.equals(otherMyClass.configuration_space_names_to_explore_)) return false;

      if (!this.exploration_range_upper_limits_.equals(otherMyClass.exploration_range_upper_limits_)) return false;

      if (!this.exploration_range_lower_limits_.equals(otherMyClass.exploration_range_lower_limits_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RigidBodyExplorationConfigurationMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("rigid_body_hash_code=");
      builder.append(this.rigid_body_hash_code_);      builder.append(", ");

      builder.append("configuration_space_names_to_explore=");
      builder.append(this.configuration_space_names_to_explore_);      builder.append(", ");

      builder.append("exploration_range_upper_limits=");
      builder.append(this.exploration_range_upper_limits_);      builder.append(", ");

      builder.append("exploration_range_lower_limits=");
      builder.append(this.exploration_range_lower_limits_);
      builder.append("}");
      return builder.toString();
   }
}
