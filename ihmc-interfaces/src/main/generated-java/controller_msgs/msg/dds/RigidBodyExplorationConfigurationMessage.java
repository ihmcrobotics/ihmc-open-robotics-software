package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message used to configure the exploration for a RRT-based planner.
 * Main usage is the IHMC WholeBodyTrajectoryToolbox.
 */
public class RigidBodyExplorationConfigurationMessage extends Packet<RigidBodyExplorationConfigurationMessage>
      implements Settable<RigidBodyExplorationConfigurationMessage>, EpsilonComparable<RigidBodyExplorationConfigurationMessage>
{
   public static final byte CONFIGURATION_SPACE_NAME_X = (byte) 0;
   public static final byte CONFIGURATION_SPACE_NAME_Y = (byte) 1;
   public static final byte CONFIGURATION_SPACE_NAME_Z = (byte) 2;
   public static final byte CONFIGURATION_SPACE_NAME_ROLL = (byte) 3;
   public static final byte CONFIGURATION_SPACE_NAME_PITCH = (byte) 4;
   public static final byte CONFIGURATION_SPACE_NAME_YAW = (byte) 5;
   public long rigid_body_name_based_hash_code_;
   public us.ihmc.idl.IDLSequence.Byte configuration_space_names_to_explore_;
   public us.ihmc.idl.IDLSequence.Double exploration_range_upper_limits_;
   public us.ihmc.idl.IDLSequence.Double exploration_range_lower_limits_;

   public RigidBodyExplorationConfigurationMessage()
   {

      configuration_space_names_to_explore_ = new us.ihmc.idl.IDLSequence.Byte(100, "type_9");

      exploration_range_upper_limits_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");

      exploration_range_lower_limits_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");
   }

   public RigidBodyExplorationConfigurationMessage(RigidBodyExplorationConfigurationMessage other)
   {
      set(other);
   }

   public void set(RigidBodyExplorationConfigurationMessage other)
   {
      rigid_body_name_based_hash_code_ = other.rigid_body_name_based_hash_code_;

      configuration_space_names_to_explore_.set(other.configuration_space_names_to_explore_);
      exploration_range_upper_limits_.set(other.exploration_range_upper_limits_);
      exploration_range_lower_limits_.set(other.exploration_range_lower_limits_);
   }

   public long getRigidBodyNameBasedHashCode()
   {
      return rigid_body_name_based_hash_code_;
   }

   public void setRigidBodyNameBasedHashCode(long rigid_body_name_based_hash_code)
   {
      rigid_body_name_based_hash_code_ = rigid_body_name_based_hash_code;
   }

   public us.ihmc.idl.IDLSequence.Byte getConfigurationSpaceNamesToExplore()
   {
      return configuration_space_names_to_explore_;
   }

   public us.ihmc.idl.IDLSequence.Double getExplorationRangeUpperLimits()
   {
      return exploration_range_upper_limits_;
   }

   public us.ihmc.idl.IDLSequence.Double getExplorationRangeLowerLimits()
   {
      return exploration_range_lower_limits_;
   }

   @Override
   public boolean epsilonEquals(RigidBodyExplorationConfigurationMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rigid_body_name_based_hash_code_, other.rigid_body_name_based_hash_code_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.configuration_space_names_to_explore_, other.configuration_space_names_to_explore_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.exploration_range_upper_limits_, other.exploration_range_upper_limits_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.exploration_range_lower_limits_, other.exploration_range_lower_limits_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof RigidBodyExplorationConfigurationMessage))
         return false;

      RigidBodyExplorationConfigurationMessage otherMyClass = (RigidBodyExplorationConfigurationMessage) other;

      if (this.rigid_body_name_based_hash_code_ != otherMyClass.rigid_body_name_based_hash_code_)
         return false;

      if (!this.configuration_space_names_to_explore_.equals(otherMyClass.configuration_space_names_to_explore_))
         return false;

      if (!this.exploration_range_upper_limits_.equals(otherMyClass.exploration_range_upper_limits_))
         return false;

      if (!this.exploration_range_lower_limits_.equals(otherMyClass.exploration_range_lower_limits_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RigidBodyExplorationConfigurationMessage {");
      builder.append("rigid_body_name_based_hash_code=");
      builder.append(this.rigid_body_name_based_hash_code_);

      builder.append(", ");
      builder.append("configuration_space_names_to_explore=");
      builder.append(this.configuration_space_names_to_explore_);

      builder.append(", ");
      builder.append("exploration_range_upper_limits=");
      builder.append(this.exploration_range_upper_limits_);

      builder.append(", ");
      builder.append("exploration_range_lower_limits=");
      builder.append(this.exploration_range_lower_limits_);

      builder.append("}");
      return builder.toString();
   }
}