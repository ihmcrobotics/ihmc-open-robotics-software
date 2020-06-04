package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message pauses the execution of a list of footsteps.
       * If this message is in the middle of executing a footstep, the robot will finish the step and pause when back in double support.
       */
public class PauseWalkingMessage extends Packet<PauseWalkingMessage> implements Settable<PauseWalkingMessage>, EpsilonComparable<PauseWalkingMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * True to pause walking, false to unpause and resume an existing footstep plan.
            */
   public boolean pause_;

   public PauseWalkingMessage()
   {



   }

   public PauseWalkingMessage(PauseWalkingMessage other)
   {
      this();
      set(other);
   }

   public void set(PauseWalkingMessage other)
   {

      sequence_id_ = other.sequence_id_;


      pause_ = other.pause_;

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
            * True to pause walking, false to unpause and resume an existing footstep plan.
            */
   public void setPause(boolean pause)
   {
      pause_ = pause;
   }
   /**
            * True to pause walking, false to unpause and resume an existing footstep plan.
            */
   public boolean getPause()
   {
      return pause_;
   }


   public static Supplier<PauseWalkingMessagePubSubType> getPubSubType()
   {
      return PauseWalkingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PauseWalkingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PauseWalkingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.pause_, other.pause_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PauseWalkingMessage)) return false;

      PauseWalkingMessage otherMyClass = (PauseWalkingMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.pause_ != otherMyClass.pause_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PauseWalkingMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("pause=");
      builder.append(this.pause_);
      builder.append("}");
      return builder.toString();
   }
}
