package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StaticHostList extends Packet<StaticHostList> implements Settable<StaticHostList>, EpsilonComparable<StaticHostList>
{
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.Host>  hosts_;

   public StaticHostList()
   {
      hosts_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.Host> (128, new us.ihmc.robotDataLogger.HostPubSubType());

   }

   public StaticHostList(StaticHostList other)
   {
      this();
      set(other);
   }

   public void set(StaticHostList other)
   {
      hosts_.set(other.hosts_);
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.Host>  getHosts()
   {
      return hosts_;
   }


   public static Supplier<StaticHostListPubSubType> getPubSubType()
   {
      return StaticHostListPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StaticHostListPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StaticHostList other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.hosts_.size() != other.hosts_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.hosts_.size(); i++)
         {  if (!this.hosts_.get(i).epsilonEquals(other.hosts_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StaticHostList)) return false;

      StaticHostList otherMyClass = (StaticHostList) other;

      if (!this.hosts_.equals(otherMyClass.hosts_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StaticHostList {");
      builder.append("hosts=");
      builder.append(this.hosts_);
      builder.append("}");
      return builder.toString();
   }
}
