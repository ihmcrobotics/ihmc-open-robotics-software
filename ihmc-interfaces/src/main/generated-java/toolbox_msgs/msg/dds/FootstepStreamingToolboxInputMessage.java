package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the FootstepStreamingToolbox API
       */
public class FootstepStreamingToolboxInputMessage extends Packet<FootstepStreamingToolboxInputMessage> implements Settable<FootstepStreamingToolboxInputMessage>, EpsilonComparable<FootstepStreamingToolboxInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing
            */
   public long sequence_id_;
   /**
            * The list of trackers used as input for the toolbox to estimate footsteps for the robot
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepStreamingToolboxTrackerMessage>  trackers_;

   public FootstepStreamingToolboxInputMessage()
   {
      trackers_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepStreamingToolboxTrackerMessage> (10, new toolbox_msgs.msg.dds.FootstepStreamingToolboxTrackerMessagePubSubType());

   }

   public FootstepStreamingToolboxInputMessage(FootstepStreamingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepStreamingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      trackers_.set(other.trackers_);
   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   /**
            * The list of trackers used as input for the toolbox to estimate footsteps for the robot
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepStreamingToolboxTrackerMessage>  getTrackers()
   {
      return trackers_;
   }


   public static Supplier<FootstepStreamingToolboxInputMessagePubSubType> getPubSubType()
   {
      return FootstepStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepStreamingToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.trackers_.size() != other.trackers_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.trackers_.size(); i++)
         {  if (!this.trackers_.get(i).epsilonEquals(other.trackers_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepStreamingToolboxInputMessage)) return false;

      FootstepStreamingToolboxInputMessage otherMyClass = (FootstepStreamingToolboxInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.trackers_.equals(otherMyClass.trackers_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepStreamingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("trackers=");
      builder.append(this.trackers_);
      builder.append("}");
      return builder.toString();
   }
}
