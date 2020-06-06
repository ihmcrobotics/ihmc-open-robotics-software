package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Send a request to change the desired PSI of the Atlas hydraulic pump.
       */
public class AtlasDesiredPumpPSIPacket extends Packet<AtlasDesiredPumpPSIPacket> implements Settable<AtlasDesiredPumpPSIPacket>, EpsilonComparable<AtlasDesiredPumpPSIPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The desired hydraulic pump PSI.
            */
   public int desired_pump_psi_;

   public AtlasDesiredPumpPSIPacket()
   {



   }

   public AtlasDesiredPumpPSIPacket(AtlasDesiredPumpPSIPacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasDesiredPumpPSIPacket other)
   {

      sequence_id_ = other.sequence_id_;


      desired_pump_psi_ = other.desired_pump_psi_;

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
            * The desired hydraulic pump PSI.
            */
   public void setDesiredPumpPsi(int desired_pump_psi)
   {
      desired_pump_psi_ = desired_pump_psi;
   }
   /**
            * The desired hydraulic pump PSI.
            */
   public int getDesiredPumpPsi()
   {
      return desired_pump_psi_;
   }


   public static Supplier<AtlasDesiredPumpPSIPacketPubSubType> getPubSubType()
   {
      return AtlasDesiredPumpPSIPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AtlasDesiredPumpPSIPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AtlasDesiredPumpPSIPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_pump_psi_, other.desired_pump_psi_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AtlasDesiredPumpPSIPacket)) return false;

      AtlasDesiredPumpPSIPacket otherMyClass = (AtlasDesiredPumpPSIPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.desired_pump_psi_ != otherMyClass.desired_pump_psi_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasDesiredPumpPSIPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("desired_pump_psi=");
      builder.append(this.desired_pump_psi_);
      builder.append("}");
      return builder.toString();
   }
}
