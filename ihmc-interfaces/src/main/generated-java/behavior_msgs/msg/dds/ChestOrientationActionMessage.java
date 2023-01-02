package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ChestOrientationActionMessage extends Packet<ChestOrientationActionMessage> implements Settable<ChestOrientationActionMessage>, EpsilonComparable<ChestOrientationActionMessage>
{
   /**
            * The orientation as a yaw-pitch-roll
            */
   public ihmc_common_msgs.msg.dds.YawPitchRollMessage orientation_;
   /**
            * The trajectory duration
            */
   public double trajectory_duration_;

   public ChestOrientationActionMessage()
   {
      orientation_ = new ihmc_common_msgs.msg.dds.YawPitchRollMessage();
   }

   public ChestOrientationActionMessage(ChestOrientationActionMessage other)
   {
      this();
      set(other);
   }

   public void set(ChestOrientationActionMessage other)
   {
      ihmc_common_msgs.msg.dds.YawPitchRollMessagePubSubType.staticCopy(other.orientation_, orientation_);
      trajectory_duration_ = other.trajectory_duration_;

   }


   /**
            * The orientation as a yaw-pitch-roll
            */
   public ihmc_common_msgs.msg.dds.YawPitchRollMessage getOrientation()
   {
      return orientation_;
   }

   /**
            * The trajectory duration
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * The trajectory duration
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }


   public static Supplier<ChestOrientationActionMessagePubSubType> getPubSubType()
   {
      return ChestOrientationActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ChestOrientationActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ChestOrientationActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ChestOrientationActionMessage)) return false;

      ChestOrientationActionMessage otherMyClass = (ChestOrientationActionMessage) other;

      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChestOrientationActionMessage {");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);
      builder.append("}");
      return builder.toString();
   }
}
