package test_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LongString" defined in "LongString_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LongString_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LongString_.idl instead.
*
*/
public class LongStringPubSubType implements us.ihmc.pubsub.TopicDataType<test_msgs.msg.dds.LongString>
{
   public static final java.lang.String name = "test_msgs::msg::dds_::LongString_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f52bb903d7647a7edb659deab69310ae7748ed507af586d22151fec7968b5c4b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(test_msgs.msg.dds.LongString data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, test_msgs.msg.dds.LongString data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (2048 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(test_msgs.msg.dds.LongString data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(test_msgs.msg.dds.LongString data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getLongString().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public static void write(test_msgs.msg.dds.LongString data, us.ihmc.idl.CDR cdr)
   {
      if(data.getLongString().size() <= 2048)
      cdr.write_type_e(data.getLongString());else
          throw new RuntimeException("long_string field exceeds the maximum length");

   }

   public static void read(test_msgs.msg.dds.LongString data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getLongString());	

   }

   @Override
   public final void serialize(test_msgs.msg.dds.LongString data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("long_string", data.getLongString());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, test_msgs.msg.dds.LongString data)
   {
      ser.read_type_e("long_string", data.getLongString());
   }

   public static void staticCopy(test_msgs.msg.dds.LongString src, test_msgs.msg.dds.LongString dest)
   {
      dest.set(src);
   }

   @Override
   public test_msgs.msg.dds.LongString createData()
   {
      return new test_msgs.msg.dds.LongString();
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
   
   public void serialize(test_msgs.msg.dds.LongString data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(test_msgs.msg.dds.LongString data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(test_msgs.msg.dds.LongString src, test_msgs.msg.dds.LongString dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LongStringPubSubType newInstance()
   {
      return new LongStringPubSubType();
   }
}
