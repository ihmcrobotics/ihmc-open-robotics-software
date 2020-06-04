package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC walking controller preview module: WalkingControllerPreviewToolbox.
       * Configure only the fields to be previewed.
       * New fields will be added to this message as the module supports them.
       */
public class WalkingControllerPreviewInputMessage extends Packet<WalkingControllerPreviewInputMessage> implements Settable<WalkingControllerPreviewInputMessage>, EpsilonComparable<WalkingControllerPreviewInputMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Set this field to request a preview of the corresponding walking sequence.
            */
   public controller_msgs.msg.dds.FootstepDataListMessage footsteps_;

   public WalkingControllerPreviewInputMessage()
   {


      footsteps_ = new controller_msgs.msg.dds.FootstepDataListMessage();

   }

   public WalkingControllerPreviewInputMessage(WalkingControllerPreviewInputMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkingControllerPreviewInputMessage other)
   {

      sequence_id_ = other.sequence_id_;


      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footsteps_, footsteps_);
   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }



   /**
            * Set this field to request a preview of the corresponding walking sequence.
            */
   public controller_msgs.msg.dds.FootstepDataListMessage getFootsteps()
   {
      return footsteps_;
   }


   public static Supplier<WalkingControllerPreviewInputMessagePubSubType> getPubSubType()
   {
      return WalkingControllerPreviewInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkingControllerPreviewInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkingControllerPreviewInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.footsteps_.epsilonEquals(other.footsteps_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkingControllerPreviewInputMessage)) return false;

      WalkingControllerPreviewInputMessage otherMyClass = (WalkingControllerPreviewInputMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.footsteps_.equals(otherMyClass.footsteps_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkingControllerPreviewInputMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("footsteps=");
      builder.append(this.footsteps_);
      builder.append("}");
      return builder.toString();
   }
}
