package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class BehaviorControlModePacket extends Packet<BehaviorControlModePacket> implements Settable<BehaviorControlModePacket>, EpsilonComparable<BehaviorControlModePacket>
{

   public static final byte STOP = (byte) 0;

   public static final byte PAUSE = (byte) 1;

   public static final byte RESUME = (byte) 2;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte behavior_control_mode_enum_request_ = (byte) 255;

   public BehaviorControlModePacket()
   {



   }

   public BehaviorControlModePacket(BehaviorControlModePacket other)
   {
      this();
      set(other);
   }

   public void set(BehaviorControlModePacket other)
   {

      sequence_id_ = other.sequence_id_;


      behavior_control_mode_enum_request_ = other.behavior_control_mode_enum_request_;

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


   public void setBehaviorControlModeEnumRequest(byte behavior_control_mode_enum_request)
   {
      behavior_control_mode_enum_request_ = behavior_control_mode_enum_request;
   }
   public byte getBehaviorControlModeEnumRequest()
   {
      return behavior_control_mode_enum_request_;
   }


   public static Supplier<BehaviorControlModePacketPubSubType> getPubSubType()
   {
      return BehaviorControlModePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorControlModePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorControlModePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.behavior_control_mode_enum_request_, other.behavior_control_mode_enum_request_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorControlModePacket)) return false;

      BehaviorControlModePacket otherMyClass = (BehaviorControlModePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.behavior_control_mode_enum_request_ != otherMyClass.behavior_control_mode_enum_request_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorControlModePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("behavior_control_mode_enum_request=");
      builder.append(this.behavior_control_mode_enum_request_);
      builder.append("}");
      return builder.toString();
   }
}
