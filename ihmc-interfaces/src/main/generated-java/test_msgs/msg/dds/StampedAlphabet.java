package test_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StampedAlphabet extends Packet<StampedAlphabet> implements Settable<StampedAlphabet>, EpsilonComparable<StampedAlphabet>
{
   public ihmc_common_msgs.msg.dds.InstantMessage last_modified_;
   public java.lang.StringBuilder alphabet_;

   public StampedAlphabet()
   {
      last_modified_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      alphabet_ = new java.lang.StringBuilder(255);
   }

   public StampedAlphabet(StampedAlphabet other)
   {
      this();
      set(other);
   }

   public void set(StampedAlphabet other)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_modified_, last_modified_);
      alphabet_.setLength(0);
      alphabet_.append(other.alphabet_);

   }


   public ihmc_common_msgs.msg.dds.InstantMessage getLastModified()
   {
      return last_modified_;
   }

   public void setAlphabet(java.lang.String alphabet)
   {
      alphabet_.setLength(0);
      alphabet_.append(alphabet);
   }

   public java.lang.String getAlphabetAsString()
   {
      return getAlphabet().toString();
   }
   public java.lang.StringBuilder getAlphabet()
   {
      return alphabet_;
   }


   public static Supplier<StampedAlphabetPubSubType> getPubSubType()
   {
      return StampedAlphabetPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StampedAlphabetPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StampedAlphabet other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.last_modified_.epsilonEquals(other.last_modified_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.alphabet_, other.alphabet_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StampedAlphabet)) return false;

      StampedAlphabet otherMyClass = (StampedAlphabet) other;

      if (!this.last_modified_.equals(otherMyClass.last_modified_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.alphabet_, otherMyClass.alphabet_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StampedAlphabet {");
      builder.append("last_modified=");
      builder.append(this.last_modified_);      builder.append(", ");
      builder.append("alphabet=");
      builder.append(this.alphabet_);
      builder.append("}");
      return builder.toString();
   }
}
