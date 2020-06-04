package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * TODO: This message is not really used, the feature it provides needs to be fixed or this message needs to be deleted.
       */
public class UIConnectedPacket extends Packet<UIConnectedPacket> implements Settable<UIConnectedPacket>, EpsilonComparable<UIConnectedPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public UIConnectedPacket()
   {


   }

   public UIConnectedPacket(UIConnectedPacket other)
   {
      this();
      set(other);
   }

   public void set(UIConnectedPacket other)
   {

      sequence_id_ = other.sequence_id_;

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


   public static Supplier<UIConnectedPacketPubSubType> getPubSubType()
   {
      return UIConnectedPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UIConnectedPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UIConnectedPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UIConnectedPacket)) return false;

      UIConnectedPacket otherMyClass = (UIConnectedPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UIConnectedPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
