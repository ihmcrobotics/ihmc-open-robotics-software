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
   		return "6ad7669ae99495429aa58b65854267e54a671108ed8ae80d647065d78c419982";
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_3(data.getValue());

      cdr.write_type_4(data.getRequestNumber());

   }

   public static void read(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setValue(cdr.read_type_3());
      	
      data.setRequestNumber(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_3("value", data.getValue());
      ser.write_type_4("request_number", data.getRequestNumber());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      data.setValue(ser.read_type_3("value"));
      data.setRequestNumber(ser.read_type_4("request_number"));
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
