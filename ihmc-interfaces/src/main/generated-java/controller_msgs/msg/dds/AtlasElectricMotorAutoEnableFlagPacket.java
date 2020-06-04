package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message.
       */
public class AtlasElectricMotorAutoEnableFlagPacket extends Packet<AtlasElectricMotorAutoEnableFlagPacket> implements Settable<AtlasElectricMotorAutoEnableFlagPacket>, EpsilonComparable<AtlasElectricMotorAutoEnableFlagPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean should_auto_enable_;

   public AtlasElectricMotorAutoEnableFlagPacket()
   {



   }

   public AtlasElectricMotorAutoEnableFlagPacket(AtlasElectricMotorAutoEnableFlagPacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasElectricMotorAutoEnableFlagPacket other)
   {

      sequence_id_ = other.sequence_id_;


      should_auto_enable_ = other.should_auto_enable_;

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


   public void setShouldAutoEnable(boolean should_auto_enable)
   {
      should_auto_enable_ = should_auto_enable;
   }
   public boolean getShouldAutoEnable()
   {
      return should_auto_enable_;
   }


   public static Supplier<AtlasElectricMotorAutoEnableFlagPacketPubSubType> getPubSubType()
   {
      return AtlasElectricMotorAutoEnableFlagPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AtlasElectricMotorAutoEnableFlagPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorAutoEnableFlagPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.should_auto_enable_, other.should_auto_enable_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AtlasElectricMotorAutoEnableFlagPacket)) return false;

      AtlasElectricMotorAutoEnableFlagPacket otherMyClass = (AtlasElectricMotorAutoEnableFlagPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.should_auto_enable_ != otherMyClass.should_auto_enable_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasElectricMotorAutoEnableFlagPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("should_auto_enable=");
      builder.append(this.should_auto_enable_);
      builder.append("}");
      return builder.toString();
   }
}
