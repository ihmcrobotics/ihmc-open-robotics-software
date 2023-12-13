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
   		return "68a1424b84c2b3598b6f9854ee0721d94884c4e3c050c753a18b31308346b563";
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getIsRequest());

      cdr.write_type_7(data.getIsConfirmation());

      cdr.write_type_4(data.getRequestNumber());

      cdr.write_type_4(data.getConfirmationNumber());

   }

   public static void read(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setIsRequest(cdr.read_type_7());
      	
      data.setIsConfirmation(cdr.read_type_7());
      	
      data.setRequestNumber(cdr.read_type_4());
      	
      data.setConfirmationNumber(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("is_request", data.getIsRequest());
      ser.write_type_7("is_confirmation", data.getIsConfirmation());
      ser.write_type_4("request_number", data.getRequestNumber());
      ser.write_type_4("confirmation_number", data.getConfirmationNumber());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      data.setIsRequest(ser.read_type_7("is_request"));
      data.setIsConfirmation(ser.read_type_7("is_confirmation"));
      data.setRequestNumber(ser.read_type_4("request_number"));
      data.setConfirmationNumber(ser.read_type_4("confirmation_number"));
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
