package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket> implements Settable<BehaviorStatusPacket>, EpsilonComparable<BehaviorStatusPacket>
{

   public static final byte NO_BEHAVIOR_RUNNING = (byte) 0;

   public static final byte BEHAVIOS_RUNNING = (byte) 1;

   public static final byte BEHAVIOR_PAUSED = (byte) 2;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte current_behavior_status_ = (byte) 255;

   public BehaviorStatusPacket()
   {



   }

   public BehaviorStatusPacket(BehaviorStatusPacket other)
   {
      this();
      set(other);
   }

   public void set(BehaviorStatusPacket other)
   {

      sequence_id_ = other.sequence_id_;


      current_behavior_status_ = other.current_behavior_status_;

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


   public void setCurrentBehaviorStatus(byte current_behavior_status)
   {
      current_behavior_status_ = current_behavior_status;
   }
   public byte getCurrentBehaviorStatus()
   {
      return current_behavior_status_;
   }


   public static Supplier<BehaviorStatusPacketPubSubType> getPubSubType()
   {
      return BehaviorStatusPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorStatusPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorStatusPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_behavior_status_, other.current_behavior_status_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorStatusPacket)) return false;

      BehaviorStatusPacket otherMyClass = (BehaviorStatusPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.current_behavior_status_ != otherMyClass.current_behavior_status_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorStatusPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("current_behavior_status=");
      builder.append(this.current_behavior_status_);
      builder.append("}");
      return builder.toString();
   }
}
