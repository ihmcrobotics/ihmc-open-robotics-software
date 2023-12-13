package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuixSideStepStateMessage" defined in "QuixSideStepStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuixSideStepStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuixSideStepStateMessage_.idl instead.
*
*/
public class QuixSideStepStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.QuixSideStepStateMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::QuixSideStepStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a811bb510946e1667eb12cdf4052d1179a69a5cb7133775ad53561c939f86e95";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getSideStepStateName());

   }

   public static void read(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setSideStepStateName(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("side_step_state_name", data.getSideStepStateName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setSideStepStateName(ser.read_type_9("side_step_state_name"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage src, exoskeleton_msgs.msg.dds.QuixSideStepStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.QuixSideStepStateMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.QuixSideStepStateMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.QuixSideStepStateMessage src, exoskeleton_msgs.msg.dds.QuixSideStepStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuixSideStepStateMessagePubSubType newInstance()
   {
      return new QuixSideStepStateMessagePubSubType();
   }
}
