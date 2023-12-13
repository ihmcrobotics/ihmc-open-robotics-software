package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FlatStepTypeMessage" defined in "FlatStepTypeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FlatStepTypeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FlatStepTypeMessage_.idl instead.
*
*/
public class FlatStepTypeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.FlatStepTypeMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::FlatStepTypeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1adf2da80a68e95174200da699a620051c4db40ee09ccbb15b1aa7880b647580";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.FlatStepTypeMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFlatStepTypeName());

   }

   public static void read(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFlatStepTypeName(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("flat_step_type_name", data.getFlatStepTypeName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.FlatStepTypeMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFlatStepTypeName(ser.read_type_9("flat_step_type_name"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.FlatStepTypeMessage src, exoskeleton_msgs.msg.dds.FlatStepTypeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.FlatStepTypeMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.FlatStepTypeMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.FlatStepTypeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.FlatStepTypeMessage src, exoskeleton_msgs.msg.dds.FlatStepTypeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FlatStepTypeMessagePubSubType newInstance()
   {
      return new FlatStepTypeMessagePubSubType();
   }
}
