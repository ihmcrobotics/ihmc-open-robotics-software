package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FoundationPoseParameters extends Packet<FoundationPoseParameters> implements Settable<FoundationPoseParameters>, EpsilonComparable<FoundationPoseParameters>
{
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_timestamp_modifiable_;
   public boolean enabled_;
   public boolean auto_reset_enabled_;
   public double reset_distance_;

   public FoundationPoseParameters()
   {
      latest_timestamp_modifiable_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
   }

   public FoundationPoseParameters(FoundationPoseParameters other)
   {
      this();
      set(other);
   }

   public void set(FoundationPoseParameters other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_timestamp_modifiable_, latest_timestamp_modifiable_);
      enabled_ = other.enabled_;

      auto_reset_enabled_ = other.auto_reset_enabled_;

      reset_distance_ = other.reset_distance_;

   }


   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestTimestampModifiable()
   {
      return latest_timestamp_modifiable_;
   }

   public void setEnabled(boolean enabled)
   {
      enabled_ = enabled;
   }
   public boolean getEnabled()
   {
      return enabled_;
   }

   public void setAutoResetEnabled(boolean auto_reset_enabled)
   {
      auto_reset_enabled_ = auto_reset_enabled;
   }
   public boolean getAutoResetEnabled()
   {
      return auto_reset_enabled_;
   }

   public void setResetDistance(double reset_distance)
   {
      reset_distance_ = reset_distance;
   }
   public double getResetDistance()
   {
      return reset_distance_;
   }


   public static Supplier<FoundationPoseParametersPubSubType> getPubSubType()
   {
      return FoundationPoseParametersPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FoundationPoseParametersPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FoundationPoseParameters other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_timestamp_modifiable_.epsilonEquals(other.latest_timestamp_modifiable_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enabled_, other.enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.auto_reset_enabled_, other.auto_reset_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.reset_distance_, other.reset_distance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FoundationPoseParameters)) return false;

      FoundationPoseParameters otherMyClass = (FoundationPoseParameters) other;

      if (!this.latest_timestamp_modifiable_.equals(otherMyClass.latest_timestamp_modifiable_)) return false;
      if(this.enabled_ != otherMyClass.enabled_) return false;

      if(this.auto_reset_enabled_ != otherMyClass.auto_reset_enabled_) return false;

      if(this.reset_distance_ != otherMyClass.reset_distance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FoundationPoseParameters {");
      builder.append("latest_timestamp_modifiable=");
      builder.append(this.latest_timestamp_modifiable_);      builder.append(", ");
      builder.append("enabled=");
      builder.append(this.enabled_);      builder.append(", ");
      builder.append("auto_reset_enabled=");
      builder.append(this.auto_reset_enabled_);      builder.append(", ");
      builder.append("reset_distance=");
      builder.append(this.reset_distance_);
      builder.append("}");
      return builder.toString();
   }
}
