package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConfirmableRequestMessage" defined in "ConfirmableRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConfirmableRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConfirmableRequestMessage_.idl instead.
*
*/
public class ConfirmableRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.ConfirmableRequestMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::ConfirmableRequestMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "77162364776eb836993a2507571a7532a546c7b1759c3d38a3b71e6e668cf3c1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (10 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (10 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRequestNumbers().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getConfirmationNumbers().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getRequestNumbers().size() <= 10)
      cdr.write_type_e(data.getRequestNumbers());else
          throw new RuntimeException("request_numbers field exceeds the maximum length");

      if(data.getConfirmationNumbers().size() <= 10)
      cdr.write_type_e(data.getConfirmationNumbers());else
          throw new RuntimeException("confirmation_numbers field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getRequestNumbers());	
      cdr.read_type_e(data.getConfirmationNumbers());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("request_numbers", data.getRequestNumbers());
      ser.write_type_e("confirmation_numbers", data.getConfirmationNumbers());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      ser.read_type_e("request_numbers", data.getRequestNumbers());
      ser.read_type_e("confirmation_numbers", data.getConfirmationNumbers());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage src, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.ConfirmableRequestMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.ConfirmableRequestMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage src, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConfirmableRequestMessagePubSubType newInstance()
   {
      return new ConfirmableRequestMessagePubSubType();
   }
}
