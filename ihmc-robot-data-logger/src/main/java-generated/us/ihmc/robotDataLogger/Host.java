package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Host extends Packet<Host> implements Settable<Host>, EpsilonComparable<Host>
{
   public java.lang.StringBuilder hostname_;
   public int port_;

   public Host()
   {
      hostname_ = new java.lang.StringBuilder(255);
   }

   public Host(Host other)
   {
      this();
      set(other);
   }

   public void set(Host other)
   {
      hostname_.setLength(0);
      hostname_.append(other.hostname_);

      port_ = other.port_;

   }

   public void setHostname(java.lang.String hostname)
   {
      hostname_.setLength(0);
      hostname_.append(hostname);
   }

   public java.lang.String getHostnameAsString()
   {
      return getHostname().toString();
   }
   public java.lang.StringBuilder getHostname()
   {
      return hostname_;
   }

   public void setPort(int port)
   {
      port_ = port;
   }
   public int getPort()
   {
      return port_;
   }


   public static Supplier<HostPubSubType> getPubSubType()
   {
      return HostPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HostPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Host other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.hostname_, other.hostname_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.port_, other.port_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Host)) return false;

      Host otherMyClass = (Host) other;

      if (!us.ihmc.idl.IDLTools.equals(this.hostname_, otherMyClass.hostname_)) return false;

      if(this.port_ != otherMyClass.port_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Host {");
      builder.append("hostname=");
      builder.append(this.hostname_);      builder.append(", ");
      builder.append("port=");
      builder.append(this.port_);
      builder.append("}");
      return builder.toString();
   }
}
