package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.TopicDataType;

import java.util.function.Supplier;

public class WholeBodyBimanipulationActionStateMessage extends Packet<WholeBodyBimanipulationActionStateMessage> implements Settable<WholeBodyBimanipulationActionStateMessage>, EpsilonComparable<WholeBodyBimanipulationActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage definition_;
   /**
            * Joint angles
            */
   public double[] joint_angles_;
   /**
            * Quality of the IK solution
            */
   public double solution_quality_;
   /**
    * Force the latest standing configuration update
    */
   public boolean force_latest_standing_configuration_update_;

   public WholeBodyBimanipulationActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage();
      joint_angles_ = new double[23];

   }

   public WholeBodyBimanipulationActionStateMessage(WholeBodyBimanipulationActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyBimanipulationActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      for(int i1 = 0; i1 < joint_angles_.length; ++i1)
      {
            joint_angles_[i1] = other.joint_angles_[i1];

      }

      solution_quality_ = other.solution_quality_;

      force_latest_standing_configuration_update_ = other.force_latest_standing_configuration_update_;

   }


   /**
    * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * Joint angles
            */
   public double[] getJointAngles()
   {
      return joint_angles_;
   }

   /**
            * Quality of the IK solution
            */
   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }
   /**
            * Quality of the IK solution
            */
   public double getSolutionQuality()
   {
      return solution_quality_;
   }

   /**
    * Force the latest standing configuration update
    */
   public void setForceLatestStandingConfigurationUpdate(boolean force_latest_standing_configuration_update)
   {
      force_latest_standing_configuration_update_ = force_latest_standing_configuration_update;
   }
   /**
    * Force the latest standing configuration update
    */
   public boolean getForceLatestStandingConfigurationUpdate()
   {
      return force_latest_standing_configuration_update_;
   }


   public static Supplier<WholeBodyBimanipulationActionStateMessagePubSubType> getPubSubType()
   {
      return WholeBodyBimanipulationActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyBimanipulationActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyBimanipulationActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      for(int i3 = 0; i3 < joint_angles_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_angles_[i3], other.joint_angles_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.force_latest_standing_configuration_update_,
                                                     other.force_latest_standing_configuration_update_,
                                                     epsilon))
         return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyBimanipulationActionStateMessage)) return false;

      WholeBodyBimanipulationActionStateMessage otherMyClass = (WholeBodyBimanipulationActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      for(int i5 = 0; i5 < joint_angles_.length; ++i5)
      {
                if(this.joint_angles_[i5] != otherMyClass.joint_angles_[i5]) return false;

      }
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;

      if (this.force_latest_standing_configuration_update_ != otherMyClass.force_latest_standing_configuration_update_)
         return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyBimanipulationActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(java.util.Arrays.toString(this.joint_angles_));      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);
      builder.append(", ");
      builder.append("force_latest_standing_configuration_update=");
      builder.append(this.force_latest_standing_configuration_update_);
      builder.append("}");
      return builder.toString();
   }
}
