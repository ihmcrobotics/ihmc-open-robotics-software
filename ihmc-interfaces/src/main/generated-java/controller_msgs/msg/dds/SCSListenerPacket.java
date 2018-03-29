package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of IHMC Simulation Construction Set. Notifies the user when the simulation
 * has stopped.
 */
public class SCSListenerPacket extends Packet<SCSListenerPacket> implements Settable<SCSListenerPacket>, EpsilonComparable<SCSListenerPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public boolean is_stopped_ = true;

   public SCSListenerPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public SCSListenerPacket(SCSListenerPacket other)
   {
      this();
      set(other);
   }

   public void set(SCSListenerPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      is_stopped_ = other.is_stopped_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setIsStopped(boolean is_stopped)
   {
      is_stopped_ = is_stopped;
   }

   public boolean getIsStopped()
   {
      return is_stopped_;
   }

   @Override
   public boolean epsilonEquals(SCSListenerPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.is_stopped_ != otherMyClass.is_stopped_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SCSListenerPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("is_stopped=");
      builder.append(this.is_stopped_);
      builder.append("}");
      return builder.toString();
   }
}
