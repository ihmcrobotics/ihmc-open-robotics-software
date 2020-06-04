package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PilotInterfaceActionPacket extends Packet<PilotInterfaceActionPacket> implements Settable<PilotInterfaceActionPacket>, EpsilonComparable<PilotInterfaceActionPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte pilot_action_;

   public PilotInterfaceActionPacket()
   {



   }

   public PilotInterfaceActionPacket(PilotInterfaceActionPacket other)
   {
      this();
      set(other);
   }

   public void set(PilotInterfaceActionPacket other)
   {

      sequence_id_ = other.sequence_id_;


      pilot_action_ = other.pilot_action_;

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


   public void setPilotAction(byte pilot_action)
   {
      pilot_action_ = pilot_action;
   }
   public byte getPilotAction()
   {
      return pilot_action_;
   }


   public static Supplier<PilotInterfaceActionPacketPubSubType> getPubSubType()
   {
      return PilotInterfaceActionPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PilotInterfaceActionPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PilotInterfaceActionPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pilot_action_, other.pilot_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PilotInterfaceActionPacket)) return false;

      PilotInterfaceActionPacket otherMyClass = (PilotInterfaceActionPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.pilot_action_ != otherMyClass.pilot_action_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PilotInterfaceActionPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("pilot_action=");
      builder.append(this.pilot_action_);
      builder.append("}");
      return builder.toString();
   }
}
