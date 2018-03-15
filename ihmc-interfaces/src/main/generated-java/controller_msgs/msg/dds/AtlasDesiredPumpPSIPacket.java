package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Send a request to change the desired PSI of the Atlas hydraulic pump.
 */
public class AtlasDesiredPumpPSIPacket implements Settable<AtlasDesiredPumpPSIPacket>, EpsilonComparable<AtlasDesiredPumpPSIPacket>
{
   /**
    * The desired hydraulic pump PSI.
    */
   private int desired_pump_psi_;

   public AtlasDesiredPumpPSIPacket()
   {
   }

   public AtlasDesiredPumpPSIPacket(AtlasDesiredPumpPSIPacket other)
   {
      set(other);
   }

   public void set(AtlasDesiredPumpPSIPacket other)
   {
      desired_pump_psi_ = other.desired_pump_psi_;
   }

   /**
    * The desired hydraulic pump PSI.
    */
   public int getDesiredPumpPsi()
   {
      return desired_pump_psi_;
   }

   /**
    * The desired hydraulic pump PSI.
    */
   public void setDesiredPumpPsi(int desired_pump_psi)
   {
      desired_pump_psi_ = desired_pump_psi;
   }

   @Override
   public boolean epsilonEquals(AtlasDesiredPumpPSIPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      if (this.desired_pump_psi_ != otherMyClass.desired_pump_psi_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasDesiredPumpPSIPacket {");
      builder.append("desired_pump_psi=");
      builder.append(this.desired_pump_psi_);

      builder.append("}");
      return builder.toString();
   }
}