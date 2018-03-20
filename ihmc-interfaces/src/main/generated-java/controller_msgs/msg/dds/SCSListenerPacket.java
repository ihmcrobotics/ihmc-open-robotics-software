package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of IHMC Simulation Construction Set.
 * Notifies the user when the simulation has stopped.
 */
public class SCSListenerPacket extends Packet<SCSListenerPacket> implements Settable<SCSListenerPacket>, EpsilonComparable<SCSListenerPacket>
{
   public boolean is_stopped_ = true;

   public SCSListenerPacket()
   {
   }

   public SCSListenerPacket(SCSListenerPacket other)
   {
      set(other);
   }

   public void set(SCSListenerPacket other)
   {
      is_stopped_ = other.is_stopped_;
   }

   public boolean getIsStopped()
   {
      return is_stopped_;
   }

   public void setIsStopped(boolean is_stopped)
   {
      is_stopped_ = is_stopped;
   }

   @Override
   public boolean epsilonEquals(SCSListenerPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_stopped_, other.is_stopped_, epsilon))
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
      if (!(other instanceof SCSListenerPacket))
         return false;

      SCSListenerPacket otherMyClass = (SCSListenerPacket) other;

      if (this.is_stopped_ != otherMyClass.is_stopped_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SCSListenerPacket {");
      builder.append("is_stopped=");
      builder.append(this.is_stopped_);

      builder.append("}");
      return builder.toString();
   }
}