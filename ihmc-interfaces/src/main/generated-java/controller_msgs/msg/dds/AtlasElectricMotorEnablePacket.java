package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Specifies a specific electric motor in the Atlas forearm to power on or off.
       */
public class AtlasElectricMotorEnablePacket extends Packet<AtlasElectricMotorEnablePacket> implements Settable<AtlasElectricMotorEnablePacket>, EpsilonComparable<AtlasElectricMotorEnablePacket>
{

   public static final byte L_ARM_WRY = (byte) 0;

   public static final byte L_ARM_WRX = (byte) 1;

   public static final byte L_ARM_WRY2 = (byte) 2;

   public static final byte R_ARM_WRY = (byte) 3;

   public static final byte R_ARM_WRX = (byte) 4;

   public static final byte R_ARM_WRY2 = (byte) 5;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The Enum value of the motor to enable
            */
   public byte atlas_electric_motor_packet_enum_enable_ = (byte) 255;

   /**
            * Boolean for enable state; true for enable, false for disable.
            */
   public boolean enable_;

   public AtlasElectricMotorEnablePacket()
   {




   }

   public AtlasElectricMotorEnablePacket(AtlasElectricMotorEnablePacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasElectricMotorEnablePacket other)
   {

      sequence_id_ = other.sequence_id_;


      atlas_electric_motor_packet_enum_enable_ = other.atlas_electric_motor_packet_enum_enable_;


      enable_ = other.enable_;

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
            * The Enum value of the motor to enable
            */
   public void setAtlasElectricMotorPacketEnumEnable(byte atlas_electric_motor_packet_enum_enable)
   {
      atlas_electric_motor_packet_enum_enable_ = atlas_electric_motor_packet_enum_enable;
   }
   /**
            * The Enum value of the motor to enable
            */
   public byte getAtlasElectricMotorPacketEnumEnable()
   {
      return atlas_electric_motor_packet_enum_enable_;
   }


   /**
            * Boolean for enable state; true for enable, false for disable.
            */
   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }
   /**
            * Boolean for enable state; true for enable, false for disable.
            */
   public boolean getEnable()
   {
      return enable_;
   }


   public static Supplier<AtlasElectricMotorEnablePacketPubSubType> getPubSubType()
   {
      return AtlasElectricMotorEnablePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AtlasElectricMotorEnablePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorEnablePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.atlas_electric_motor_packet_enum_enable_, other.atlas_electric_motor_packet_enum_enable_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_, other.enable_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AtlasElectricMotorEnablePacket)) return false;

      AtlasElectricMotorEnablePacket otherMyClass = (AtlasElectricMotorEnablePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.atlas_electric_motor_packet_enum_enable_ != otherMyClass.atlas_electric_motor_packet_enum_enable_) return false;


      if(this.enable_ != otherMyClass.enable_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasElectricMotorEnablePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("atlas_electric_motor_packet_enum_enable=");
      builder.append(this.atlas_electric_motor_packet_enum_enable_);      builder.append(", ");

      builder.append("enable=");
      builder.append(this.enable_);
      builder.append("}");
      return builder.toString();
   }
}
