package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "UUIDMessage" defined in "UUIDMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from UUIDMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit UUIDMessage_.idl instead.
*
*/
public class UUIDMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.UUIDMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::UUIDMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "380ebd2d28324312d785d8a39c840b1956fa7951e3cce8aeba7713a868ab0572";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.UUIDMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.UUIDMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.UUIDMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.UUIDMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.UUIDMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getLeastSignificantBits());

      cdr.write_type_11(data.getMostSignificantBits());

   }

   public static void read(ihmc_common_msgs.msg.dds.UUIDMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setLeastSignificantBits(cdr.read_type_11());
      	
      data.setMostSignificantBits(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.UUIDMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("least_significant_bits", data.getLeastSignificantBits());
      ser.write_type_11("most_significant_bits", data.getMostSignificantBits());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.UUIDMessage data)
   {
      data.setLeastSignificantBits(ser.read_type_11("least_significant_bits"));
      data.setMostSignificantBits(ser.read_type_11("most_significant_bits"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.UUIDMessage src, ihmc_common_msgs.msg.dds.UUIDMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.UUIDMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.UUIDMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.UUIDMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.UUIDMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.UUIDMessage src, ihmc_common_msgs.msg.dds.UUIDMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public UUIDMessagePubSubType newInstance()
   {
      return new UUIDMessagePubSubType();
   }
}
