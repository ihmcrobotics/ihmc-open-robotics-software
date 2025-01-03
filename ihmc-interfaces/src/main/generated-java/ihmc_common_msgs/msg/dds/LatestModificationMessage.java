package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message to hold onto the most recent modification timestamp
       * on the data this message is associated with.
       */
public class LatestModificationMessage extends Packet<LatestModificationMessage> implements Settable<LatestModificationMessage>, EpsilonComparable<LatestModificationMessage>
{
   /**
            * The guid of the peer that made the most recent modification
            */
   public ihmc_common_msgs.msg.dds.GuidMessage latest_modifier_id_;
   /**
            * The time the latest modification was made in peer frame
            */
   public ihmc_common_msgs.msg.dds.InstantMessage latest_modification_time_in_modifier_frame_;
   /**
            * Modification number used to order the modifications across all peers
            * This allows us to workaround peer clock offset drift.
            */
   public long latest_modification_number_;
   /**
            * Human readable name of the latest modifier
            */
   public java.lang.StringBuilder latest_modifier_name_;
   /**
            * A way for a peer to request the full data to be resent
            */
   public boolean full_data_needed_;

   public LatestModificationMessage()
   {
      latest_modifier_id_ = new ihmc_common_msgs.msg.dds.GuidMessage();
      latest_modification_time_in_modifier_frame_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      latest_modifier_name_ = new java.lang.StringBuilder(255);
   }

   public LatestModificationMessage(LatestModificationMessage other)
   {
      this();
      set(other);
   }

   public void set(LatestModificationMessage other)
   {
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.staticCopy(other.latest_modifier_id_, latest_modifier_id_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.latest_modification_time_in_modifier_frame_, latest_modification_time_in_modifier_frame_);
      latest_modification_number_ = other.latest_modification_number_;

      latest_modifier_name_.setLength(0);
      latest_modifier_name_.append(other.latest_modifier_name_);

      full_data_needed_ = other.full_data_needed_;

   }


   /**
            * The guid of the peer that made the most recent modification
            */
   public ihmc_common_msgs.msg.dds.GuidMessage getLatestModifierId()
   {
      return latest_modifier_id_;
   }


   /**
            * The time the latest modification was made in peer frame
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getLatestModificationTimeInModifierFrame()
   {
      return latest_modification_time_in_modifier_frame_;
   }

   /**
            * Modification number used to order the modifications across all peers
            * This allows us to workaround peer clock offset drift.
            */
   public void setLatestModificationNumber(long latest_modification_number)
   {
      latest_modification_number_ = latest_modification_number;
   }
   /**
            * Modification number used to order the modifications across all peers
            * This allows us to workaround peer clock offset drift.
            */
   public long getLatestModificationNumber()
   {
      return latest_modification_number_;
   }

   /**
            * Human readable name of the latest modifier
            */
   public void setLatestModifierName(java.lang.String latest_modifier_name)
   {
      latest_modifier_name_.setLength(0);
      latest_modifier_name_.append(latest_modifier_name);
   }

   /**
            * Human readable name of the latest modifier
            */
   public java.lang.String getLatestModifierNameAsString()
   {
      return getLatestModifierName().toString();
   }
   /**
            * Human readable name of the latest modifier
            */
   public java.lang.StringBuilder getLatestModifierName()
   {
      return latest_modifier_name_;
   }

   /**
            * A way for a peer to request the full data to be resent
            */
   public void setFullDataNeeded(boolean full_data_needed)
   {
      full_data_needed_ = full_data_needed;
   }
   /**
            * A way for a peer to request the full data to be resent
            */
   public boolean getFullDataNeeded()
   {
      return full_data_needed_;
   }


   public static Supplier<LatestModificationMessagePubSubType> getPubSubType()
   {
      return LatestModificationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LatestModificationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LatestModificationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_modifier_id_.epsilonEquals(other.latest_modifier_id_, epsilon)) return false;
      if (!this.latest_modification_time_in_modifier_frame_.epsilonEquals(other.latest_modification_time_in_modifier_frame_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.latest_modification_number_, other.latest_modification_number_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.latest_modifier_name_, other.latest_modifier_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.full_data_needed_, other.full_data_needed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LatestModificationMessage)) return false;

      LatestModificationMessage otherMyClass = (LatestModificationMessage) other;

      if (!this.latest_modifier_id_.equals(otherMyClass.latest_modifier_id_)) return false;
      if (!this.latest_modification_time_in_modifier_frame_.equals(otherMyClass.latest_modification_time_in_modifier_frame_)) return false;
      if(this.latest_modification_number_ != otherMyClass.latest_modification_number_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.latest_modifier_name_, otherMyClass.latest_modifier_name_)) return false;

      if(this.full_data_needed_ != otherMyClass.full_data_needed_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LatestModificationMessage {");
      builder.append("latest_modifier_id=");
      builder.append(this.latest_modifier_id_);      builder.append(", ");
      builder.append("latest_modification_time_in_modifier_frame=");
      builder.append(this.latest_modification_time_in_modifier_frame_);      builder.append(", ");
      builder.append("latest_modification_number=");
      builder.append(this.latest_modification_number_);      builder.append(", ");
      builder.append("latest_modifier_name=");
      builder.append(this.latest_modifier_name_);      builder.append(", ");
      builder.append("full_data_needed=");
      builder.append(this.full_data_needed_);
      builder.append("}");
      return builder.toString();
   }
}
