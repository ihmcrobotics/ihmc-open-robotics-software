package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message is used to switch the control scheme between different stepping modes.
       */
public class QuadrupedRequestedSteppingStateMessage extends Packet<QuadrupedRequestedSteppingStateMessage> implements Settable<QuadrupedRequestedSteppingStateMessage>, EpsilonComparable<QuadrupedRequestedSteppingStateMessage>
{

   public static final byte REQUEST_STAND = (byte) 0;

   public static final byte REQUEST_STEP = (byte) 1;

   public static final byte REQUEST_SOLE_WAYPOINT = (byte) 2;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies the which state the controller should transition into.
            */
   public byte quadruped_stepping_requested_event_ = (byte) 255;

   public QuadrupedRequestedSteppingStateMessage()
   {



   }

   public QuadrupedRequestedSteppingStateMessage(QuadrupedRequestedSteppingStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedRequestedSteppingStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      quadruped_stepping_requested_event_ = other.quadruped_stepping_requested_event_;

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
            * Specifies the which state the controller should transition into.
            */
   public void setQuadrupedSteppingRequestedEvent(byte quadruped_stepping_requested_event)
   {
      quadruped_stepping_requested_event_ = quadruped_stepping_requested_event;
   }
   /**
            * Specifies the which state the controller should transition into.
            */
   public byte getQuadrupedSteppingRequestedEvent()
   {
      return quadruped_stepping_requested_event_;
   }


   public static Supplier<QuadrupedRequestedSteppingStateMessagePubSubType> getPubSubType()
   {
      return QuadrupedRequestedSteppingStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedRequestedSteppingStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedRequestedSteppingStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quadruped_stepping_requested_event_, other.quadruped_stepping_requested_event_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedRequestedSteppingStateMessage)) return false;

      QuadrupedRequestedSteppingStateMessage otherMyClass = (QuadrupedRequestedSteppingStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.quadruped_stepping_requested_event_ != otherMyClass.quadruped_stepping_requested_event_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedRequestedSteppingStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("quadruped_stepping_requested_event=");
      builder.append(this.quadruped_stepping_requested_event_);
      builder.append("}");
      return builder.toString();
   }
}
