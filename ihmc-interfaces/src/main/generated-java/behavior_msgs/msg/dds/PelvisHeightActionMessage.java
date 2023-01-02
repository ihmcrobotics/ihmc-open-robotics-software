package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PelvisHeightActionMessage extends Packet<PelvisHeightActionMessage> implements Settable<PelvisHeightActionMessage>, EpsilonComparable<PelvisHeightActionMessage>
{
   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage action_information_;
   /**
            * Duration of the trajectory
            */
   public double trajectory_duration_;
   /**
            * Z height in world frame
            */
   public double height_in_world_;

   public PelvisHeightActionMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
   }

   public PelvisHeightActionMessage(PelvisHeightActionMessage other)
   {
      this();
      set(other);
   }

   public void set(PelvisHeightActionMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      trajectory_duration_ = other.trajectory_duration_;

      height_in_world_ = other.height_in_world_;

   }


   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage getActionInformation()
   {
      return action_information_;
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

   /**
            * Z height in world frame
            */
   public void setHeightInWorld(double height_in_world)
   {
      height_in_world_ = height_in_world;
   }
   /**
            * Z height in world frame
            */
   public double getHeightInWorld()
   {
      return height_in_world_;
   }


   public static Supplier<PelvisHeightActionMessagePubSubType> getPubSubType()
   {
      return PelvisHeightActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PelvisHeightActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PelvisHeightActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_in_world_, other.height_in_world_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PelvisHeightActionMessage)) return false;

      PelvisHeightActionMessage otherMyClass = (PelvisHeightActionMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.height_in_world_ != otherMyClass.height_in_world_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisHeightActionMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("height_in_world=");
      builder.append(this.height_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
