package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandPoseActionStateMessage extends Packet<HandPoseActionStateMessage> implements Settable<HandPoseActionStateMessage>, EpsilonComparable<HandPoseActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.HandPoseActionDefinitionMessage definition_;
   /**
            * This is the estimated goal chest frame as the robot executes a potential whole body action.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage goal_chest_transform_to_world_;
   /**
            * Hand force
            */
   public us.ihmc.euclid.tuple3D.Vector3D force_;
   /**
            * Hand torque
            */
   public us.ihmc.euclid.tuple3D.Vector3D torque_;
   /**
            * Joint angles
            */
   public double[] joint_angles_;
   /**
            * Quality of the IK solution
            */
   public double solution_quality_;

   public HandPoseActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.HandPoseActionDefinitionMessage();
      goal_chest_transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      force_ = new us.ihmc.euclid.tuple3D.Vector3D();
      torque_ = new us.ihmc.euclid.tuple3D.Vector3D();
      joint_angles_ = new double[7];

   }

   public HandPoseActionStateMessage(HandPoseActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(HandPoseActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.goal_chest_transform_to_world_, goal_chest_transform_to_world_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.force_, force_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.torque_, torque_);
      for(int i1 = 0; i1 < joint_angles_.length; ++i1)
      {
            joint_angles_[i1] = other.joint_angles_[i1];

      }

      solution_quality_ = other.solution_quality_;

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
   public behavior_msgs.msg.dds.HandPoseActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * This is the estimated goal chest frame as the robot executes a potential whole body action.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getGoalChestTransformToWorld()
   {
      return goal_chest_transform_to_world_;
   }


   /**
            * Hand force
            */
   public us.ihmc.euclid.tuple3D.Vector3D getForce()
   {
      return force_;
   }


   /**
            * Hand torque
            */
   public us.ihmc.euclid.tuple3D.Vector3D getTorque()
   {
      return torque_;
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


   public static Supplier<HandPoseActionStateMessagePubSubType> getPubSubType()
   {
      return HandPoseActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandPoseActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandPoseActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!this.goal_chest_transform_to_world_.epsilonEquals(other.goal_chest_transform_to_world_, epsilon)) return false;
      if (!this.force_.epsilonEquals(other.force_, epsilon)) return false;
      if (!this.torque_.epsilonEquals(other.torque_, epsilon)) return false;
      for(int i3 = 0; i3 < joint_angles_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_angles_[i3], other.joint_angles_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandPoseActionStateMessage)) return false;

      HandPoseActionStateMessage otherMyClass = (HandPoseActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.goal_chest_transform_to_world_.equals(otherMyClass.goal_chest_transform_to_world_)) return false;
      if (!this.force_.equals(otherMyClass.force_)) return false;
      if (!this.torque_.equals(otherMyClass.torque_)) return false;
      for(int i5 = 0; i5 < joint_angles_.length; ++i5)
      {
                if(this.joint_angles_[i5] != otherMyClass.joint_angles_[i5]) return false;

      }
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandPoseActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("goal_chest_transform_to_world=");
      builder.append(this.goal_chest_transform_to_world_);      builder.append(", ");
      builder.append("force=");
      builder.append(this.force_);      builder.append(", ");
      builder.append("torque=");
      builder.append(this.torque_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(java.util.Arrays.toString(this.joint_angles_));      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);
      builder.append("}");
      return builder.toString();
   }
}
