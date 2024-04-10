package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DoorTraversalStateMessage extends Packet<DoorTraversalStateMessage> implements Settable<DoorTraversalStateMessage>, EpsilonComparable<DoorTraversalStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.DoorTraversalDefinitionMessage definition_;
   public double door_hinge_joint_angle_;
   public double right_knuckle_x1_;
   public double right_knuckle_x2_;

   public DoorTraversalStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.DoorTraversalDefinitionMessage();
   }

   public DoorTraversalStateMessage(DoorTraversalStateMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorTraversalStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      door_hinge_joint_angle_ = other.door_hinge_joint_angle_;

      right_knuckle_x1_ = other.right_knuckle_x1_;

      right_knuckle_x2_ = other.right_knuckle_x2_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.DoorTraversalDefinitionMessage getDefinition()
   {
      return definition_;
   }

   public void setDoorHingeJointAngle(double door_hinge_joint_angle)
   {
      door_hinge_joint_angle_ = door_hinge_joint_angle;
   }
   public double getDoorHingeJointAngle()
   {
      return door_hinge_joint_angle_;
   }

   public void setRightKnuckleX1(double right_knuckle_x1)
   {
      right_knuckle_x1_ = right_knuckle_x1;
   }
   public double getRightKnuckleX1()
   {
      return right_knuckle_x1_;
   }

   public void setRightKnuckleX2(double right_knuckle_x2)
   {
      right_knuckle_x2_ = right_knuckle_x2;
   }
   public double getRightKnuckleX2()
   {
      return right_knuckle_x2_;
   }


   public static Supplier<DoorTraversalStateMessagePubSubType> getPubSubType()
   {
      return DoorTraversalStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorTraversalStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorTraversalStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.door_hinge_joint_angle_, other.door_hinge_joint_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_knuckle_x1_, other.right_knuckle_x1_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_knuckle_x2_, other.right_knuckle_x2_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorTraversalStateMessage)) return false;

      DoorTraversalStateMessage otherMyClass = (DoorTraversalStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.door_hinge_joint_angle_ != otherMyClass.door_hinge_joint_angle_) return false;

      if(this.right_knuckle_x1_ != otherMyClass.right_knuckle_x1_) return false;

      if(this.right_knuckle_x2_ != otherMyClass.right_knuckle_x2_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorTraversalStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("door_hinge_joint_angle=");
      builder.append(this.door_hinge_joint_angle_);      builder.append(", ");
      builder.append("right_knuckle_x1=");
      builder.append(this.right_knuckle_x1_);      builder.append(", ");
      builder.append("right_knuckle_x2=");
      builder.append(this.right_knuckle_x2_);
      builder.append("}");
      return builder.toString();
   }
}
