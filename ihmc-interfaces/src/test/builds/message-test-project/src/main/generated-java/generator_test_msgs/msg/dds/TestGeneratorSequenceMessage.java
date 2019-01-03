package generator_test_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TestGeneratorSequenceMessage extends Packet<TestGeneratorSequenceMessage> implements Settable<TestGeneratorSequenceMessage>, EpsilonComparable<TestGeneratorSequenceMessage>
{
   public long sequence_id_;

   public TestGeneratorSequenceMessage()
   {
   }

   public TestGeneratorSequenceMessage(TestGeneratorSequenceMessage other)
   {
      this();
      set(other);
   }

   public void set(TestGeneratorSequenceMessage other)
   {
      sequence_id_ = other.sequence_id_;

   }

   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public static Supplier<TestGeneratorSequenceMessagePubSubType> getPubSubType()
   {
      return TestGeneratorSequenceMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TestGeneratorSequenceMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TestGeneratorSequenceMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TestGeneratorSequenceMessage)) return false;

      TestGeneratorSequenceMessage otherMyClass = (TestGeneratorSequenceMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TestGeneratorSequenceMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
