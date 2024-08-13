package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PrimitiveDataVectorMessage" defined in "PrimitiveDataVectorMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PrimitiveDataVectorMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PrimitiveDataVectorMessage_.idl instead.
*
*/
public class PrimitiveDataVectorMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::PrimitiveDataVectorMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "09642cb7ad6c5f27a8b441af43d779a48af661d1096d33a0520f5a32c8d9a52d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (200 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (200 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (200 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDoubleValues().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getIntegerValues().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBooleanValues().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDoubleValues().size() <= 200)
      cdr.write_type_e(data.getDoubleValues());else
          throw new RuntimeException("double_values field exceeds the maximum length");

      if(data.getIntegerValues().size() <= 200)
      cdr.write_type_e(data.getIntegerValues());else
          throw new RuntimeException("integer_values field exceeds the maximum length");

      if(data.getBooleanValues().size() <= 200)
      cdr.write_type_e(data.getBooleanValues());else
          throw new RuntimeException("boolean_values field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getDoubleValues());	
      cdr.read_type_e(data.getIntegerValues());	
      cdr.read_type_e(data.getBooleanValues());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("double_values", data.getDoubleValues());
      ser.write_type_e("integer_values", data.getIntegerValues());
      ser.write_type_e("boolean_values", data.getBooleanValues());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data)
   {
      ser.read_type_e("double_values", data.getDoubleValues());
      ser.read_type_e("integer_values", data.getIntegerValues());
      ser.read_type_e("boolean_values", data.getBooleanValues());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage src, ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage src, ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PrimitiveDataVectorMessagePubSubType newInstance()
   {
      return new PrimitiveDataVectorMessagePubSubType();
   }
}
