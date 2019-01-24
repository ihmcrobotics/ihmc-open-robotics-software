package generator_test_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TestGeneratorSequenceMessage" defined in "TestGeneratorSequenceMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TestGeneratorSequenceMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TestGeneratorSequenceMessage_.idl instead.
*
*/
public class TestGeneratorSequenceMessagePubSubType implements us.ihmc.pubsub.TopicDataType<generator_test_msgs.msg.dds.TestGeneratorSequenceMessage>
{
   public static final java.lang.String name = "generator_test_msgs::msg::dds_::TestGeneratorSequenceMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

   }

   public static void read(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));   }

   public static void staticCopy(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage src, generator_test_msgs.msg.dds.TestGeneratorSequenceMessage dest)
   {
      dest.set(src);
   }

   @Override
   public generator_test_msgs.msg.dds.TestGeneratorSequenceMessage createData()
   {
      return new generator_test_msgs.msg.dds.TestGeneratorSequenceMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(generator_test_msgs.msg.dds.TestGeneratorSequenceMessage src, generator_test_msgs.msg.dds.TestGeneratorSequenceMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TestGeneratorSequenceMessagePubSubType newInstance()
   {
      return new TestGeneratorSequenceMessagePubSubType();
   }
}
