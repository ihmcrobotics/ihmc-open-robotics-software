package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandActionStateMessage extends Packet<AbilityHandActionStateMessage> implements Settable<AbilityHandActionStateMessage>, EpsilonComparable<AbilityHandActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage definition_;
   /**
            * Current finger positions
            */
   public float[] current_finger_positions_;
   /**
            * Desired finger positions
            */
   public float[] desired_finger_positions_;

   public AbilityHandActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage();
      current_finger_positions_ = new float[6];

      desired_finger_positions_ = new float[6];

   }

   public AbilityHandActionStateMessage(AbilityHandActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      for(int i1 = 0; i1 < current_finger_positions_.length; ++i1)
      {
            current_finger_positions_[i1] = other.current_finger_positions_[i1];

      }

      for(int i3 = 0; i3 < desired_finger_positions_.length; ++i3)
      {
            desired_finger_positions_[i3] = other.desired_finger_positions_[i3];

      }

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
   public behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * Current finger positions
            */
   public float[] getCurrentFingerPositions()
   {
      return current_finger_positions_;
   }


   /**
            * Desired finger positions
            */
   public float[] getDesiredFingerPositions()
   {
      return desired_finger_positions_;
   }


   public static Supplier<AbilityHandActionStateMessagePubSubType> getPubSubType()
   {
      return AbilityHandActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      for(int i5 = 0; i5 < current_finger_positions_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_finger_positions_[i5], other.current_finger_positions_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < desired_finger_positions_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_finger_positions_[i7], other.desired_finger_positions_[i7], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandActionStateMessage)) return false;

      AbilityHandActionStateMessage otherMyClass = (AbilityHandActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      for(int i9 = 0; i9 < current_finger_positions_.length; ++i9)
      {
                if(this.current_finger_positions_[i9] != otherMyClass.current_finger_positions_[i9]) return false;

      }
      for(int i11 = 0; i11 < desired_finger_positions_.length; ++i11)
      {
                if(this.desired_finger_positions_[i11] != otherMyClass.desired_finger_positions_[i11]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("current_finger_positions=");
      builder.append(java.util.Arrays.toString(this.current_finger_positions_));      builder.append(", ");
      builder.append("desired_finger_positions=");
      builder.append(java.util.Arrays.toString(this.desired_finger_positions_));
      builder.append("}");
      return builder.toString();
   }
}
