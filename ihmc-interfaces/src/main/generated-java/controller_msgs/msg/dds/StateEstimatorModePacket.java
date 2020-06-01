package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       */
public class StateEstimatorModePacket extends Packet<StateEstimatorModePacket> implements Settable<StateEstimatorModePacket>, EpsilonComparable<StateEstimatorModePacket>
{

   public static final byte NORMAL = (byte) 0;

   public static final byte FROZEN = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte requested_state_estimator_mode_ = (byte) 255;

   public StateEstimatorModePacket()
   {



   }

   public StateEstimatorModePacket(StateEstimatorModePacket other)
   {
      this();
      set(other);
   }

   public void set(StateEstimatorModePacket other)
   {

      sequence_id_ = other.sequence_id_;


      requested_state_estimator_mode_ = other.requested_state_estimator_mode_;

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


   public void setRequestedStateEstimatorMode(byte requested_state_estimator_mode)
   {
      requested_state_estimator_mode_ = requested_state_estimator_mode;
   }
   public byte getRequestedStateEstimatorMode()
   {
      return requested_state_estimator_mode_;
   }


   public static Supplier<StateEstimatorModePacketPubSubType> getPubSubType()
   {
      return StateEstimatorModePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StateEstimatorModePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StateEstimatorModePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_state_estimator_mode_, other.requested_state_estimator_mode_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StateEstimatorModePacket)) return false;

      StateEstimatorModePacket otherMyClass = (StateEstimatorModePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.requested_state_estimator_mode_ != otherMyClass.requested_state_estimator_mode_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StateEstimatorModePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("requested_state_estimator_mode=");
      builder.append(this.requested_state_estimator_mode_);
      builder.append("}");
      return builder.toString();
   }
}
