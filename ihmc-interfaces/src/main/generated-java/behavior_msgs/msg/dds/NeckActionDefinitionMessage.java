package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class NeckActionDefinitionMessage extends Packet<NeckActionDefinitionMessage> implements Settable<NeckActionDefinitionMessage>, EpsilonComparable<NeckActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Neck pitch angle
            */
   public double pitch_;
   /**
            * Neck yaw angle
            */
   public double yaw_;
   /**
            * Duration of the trajectory
            */
   public double trajectory_duration_;

   public NeckActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
   }

   public NeckActionDefinitionMessage(NeckActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(NeckActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      pitch_ = other.pitch_;

      yaw_ = other.yaw_;

      trajectory_duration_ = other.trajectory_duration_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Neck pitch angle
            */
   public void setPitch(double pitch)
   {
      pitch_ = pitch;
   }
   /**
            * Neck pitch angle
            */
   public double getPitch()
   {
      return pitch_;
   }

   /**
            * Neck yaw angle
            */
   public void setYaw(double yaw)
   {
      yaw_ = yaw;
   }
   /**
            * Neck yaw angle
            */
   public double getYaw()
   {
      return yaw_;
   }

   /**
            * Duration of the trajectory
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * Duration of the trajectory
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }


   public static Supplier<NeckActionDefinitionMessagePubSubType> getPubSubType()
   {
      return NeckActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return NeckActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(NeckActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pitch_, other.pitch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_, other.yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof NeckActionDefinitionMessage)) return false;

      NeckActionDefinitionMessage otherMyClass = (NeckActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.pitch_ != otherMyClass.pitch_) return false;

      if(this.yaw_ != otherMyClass.yaw_) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NeckActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("pitch=");
      builder.append(this.pitch_);      builder.append(", ");
      builder.append("yaw=");
      builder.append(this.yaw_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);
      builder.append("}");
      return builder.toString();
   }
}
