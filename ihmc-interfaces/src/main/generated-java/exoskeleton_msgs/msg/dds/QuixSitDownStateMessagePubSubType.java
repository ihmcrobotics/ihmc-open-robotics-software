package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuixSitDownStateMessage" defined in "QuixSitDownStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuixSitDownStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuixSitDownStateMessage_.idl instead.
*
*/
public class QuixSitDownStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.QuixSitDownStateMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::QuixSitDownStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2ae0ff8f30f1f466fbcbd533c46e728186d16b1c3c8921eeef9b2c7bff517207";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getSitDownStateName());

   }

   public static void read(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setSitDownStateName(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("sit_down_state_name", data.getSitDownStateName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setSitDownStateName(ser.read_type_9("sit_down_state_name"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage src, exoskeleton_msgs.msg.dds.QuixSitDownStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.QuixSitDownStateMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.QuixSitDownStateMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.QuixSitDownStateMessage src, exoskeleton_msgs.msg.dds.QuixSitDownStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuixSitDownStateMessagePubSubType newInstance()
   {
      return new QuixSitDownStateMessagePubSubType();
   }
}
