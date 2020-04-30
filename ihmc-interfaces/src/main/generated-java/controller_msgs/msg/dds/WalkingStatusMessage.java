package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * The controller sends this message to notify the user of the status of walking.
       */
public class WalkingStatusMessage extends Packet<WalkingStatusMessage> implements Settable<WalkingStatusMessage>, EpsilonComparable<WalkingStatusMessage>
{

   /**
          * The robot has begun its initial transfer/sway at the start of a walking plan.
          */
   public static final byte STARTED = (byte) 0;

   /**
          * The robot has finished its final transfer/sway at the end of a walking plan.
          */
   public static final byte COMPLETED = (byte) 1;

   /**
          * A walking abort has been requested by the controller.
          */
   public static final byte ABORT_REQUESTED = (byte) 2;

   /**
          * The robot is back to standing on a break waiting for either an un-pause command or new footsteps.
          */
   public static final byte PAUSED = (byte) 3;

   /**
          * The robot is resuming the series of footsteps that were paused.
          */
   public static final byte RESUMED = (byte) 4;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Status of walking.
            */
   public byte walking_status_ = (byte) 255;

   public WalkingStatusMessage()
   {



   }

   public WalkingStatusMessage(WalkingStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkingStatusMessage other)
   {

      sequence_id_ = other.sequence_id_;


      walking_status_ = other.walking_status_;

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
            * Status of walking.
            */
   public void setWalkingStatus(byte walking_status)
   {
      walking_status_ = walking_status;
   }
   /**
            * Status of walking.
            */
   public byte getWalkingStatus()
   {
      return walking_status_;
   }


   public static Supplier<WalkingStatusMessagePubSubType> getPubSubType()
   {
      return WalkingStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkingStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkingStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.walking_status_, other.walking_status_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkingStatusMessage)) return false;

      WalkingStatusMessage otherMyClass = (WalkingStatusMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.walking_status_ != otherMyClass.walking_status_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkingStatusMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("walking_status=");
      builder.append(this.walking_status_);
      builder.append("}");
      return builder.toString();
   }
}
