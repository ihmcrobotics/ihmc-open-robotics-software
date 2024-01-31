package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ScrewPrimitiveActionStateMessage extends Packet<ScrewPrimitiveActionStateMessage> implements Settable<ScrewPrimitiveActionStateMessage>, EpsilonComparable<ScrewPrimitiveActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage definition_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  trajectory_;
   /**
            * Hand force
            */
   public us.ihmc.euclid.tuple3D.Vector3D force_;
   /**
            * Hand torque
            */
   public us.ihmc.euclid.tuple3D.Vector3D torque_;

   public ScrewPrimitiveActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage();
      trajectory_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (50, new geometry_msgs.msg.dds.PosePubSubType());
      force_ = new us.ihmc.euclid.tuple3D.Vector3D();
      torque_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public ScrewPrimitiveActionStateMessage(ScrewPrimitiveActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ScrewPrimitiveActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      trajectory_.set(other.trajectory_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.force_, force_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.torque_, torque_);
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
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getTrajectory()
   {
      return trajectory_;
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


   public static Supplier<ScrewPrimitiveActionStateMessagePubSubType> getPubSubType()
   {
      return ScrewPrimitiveActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ScrewPrimitiveActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ScrewPrimitiveActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (this.trajectory_.size() != other.trajectory_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.trajectory_.size(); i++)
         {  if (!this.trajectory_.get(i).epsilonEquals(other.trajectory_.get(i), epsilon)) return false; }
      }

      if (!this.force_.epsilonEquals(other.force_, epsilon)) return false;
      if (!this.torque_.epsilonEquals(other.torque_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ScrewPrimitiveActionStateMessage)) return false;

      ScrewPrimitiveActionStateMessage otherMyClass = (ScrewPrimitiveActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.trajectory_.equals(otherMyClass.trajectory_)) return false;
      if (!this.force_.equals(otherMyClass.force_)) return false;
      if (!this.torque_.equals(otherMyClass.torque_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ScrewPrimitiveActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("trajectory=");
      builder.append(this.trajectory_);      builder.append(", ");
      builder.append("force=");
      builder.append(this.force_);      builder.append(", ");
      builder.append("torque=");
      builder.append(this.torque_);
      builder.append("}");
      return builder.toString();
   }
}
