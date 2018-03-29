package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Send a request to change the desired PSI of the Atlas hydraulic pump.
 */
public class AtlasDesiredPumpPSIPacket extends Packet<AtlasDesiredPumpPSIPacket>
      implements Settable<AtlasDesiredPumpPSIPacket>, EpsilonComparable<AtlasDesiredPumpPSIPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * The desired hydraulic pump PSI.
    */
   public int desired_pump_psi_;

   public AtlasDesiredPumpPSIPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public AtlasDesiredPumpPSIPacket(AtlasDesiredPumpPSIPacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasDesiredPumpPSIPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      desired_pump_psi_ = other.desired_pump_psi_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
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

   @Override
   public boolean epsilonEquals(AtlasDesiredPumpPSIPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_pump_psi_, other.desired_pump_psi_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof AtlasDesiredPumpPSIPacket))
         return false;

      AtlasDesiredPumpPSIPacket otherMyClass = (AtlasDesiredPumpPSIPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.desired_pump_psi_ != otherMyClass.desired_pump_psi_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasDesiredPumpPSIPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("desired_pump_psi=");
      builder.append(this.desired_pump_psi_);
      builder.append("}");
      return builder.toString();
   }
}
