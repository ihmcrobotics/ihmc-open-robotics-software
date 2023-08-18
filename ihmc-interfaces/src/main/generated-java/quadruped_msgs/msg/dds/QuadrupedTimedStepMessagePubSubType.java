package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedTimedStepMessage" defined in "QuadrupedTimedStepMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedTimedStepMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedTimedStepMessage_.idl instead.
*
*/
public class QuadrupedTimedStepMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedTimedStepMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedTimedStepMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "456c6fe607adf255958acab6117328467864e5e6717086063e5af8c4a7ebb891";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedStepMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getTimeInterval(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedStepMessagePubSubType.getCdrSerializedSize(data.getQuadrupedStepMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.write(data.getTimeInterval(), cdr);
      quadruped_msgs.msg.dds.QuadrupedStepMessagePubSubType.write(data.getQuadrupedStepMessage(), cdr);
   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.read(data.getTimeInterval(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedStepMessagePubSubType.read(data.getQuadrupedStepMessage(), cdr);	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("time_interval", new ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType(), data.getTimeInterval());

      ser.write_type_a("quadruped_step_message", new quadruped_msgs.msg.dds.QuadrupedStepMessagePubSubType(), data.getQuadrupedStepMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("time_interval", new ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType(), data.getTimeInterval());

      ser.read_type_a("quadruped_step_message", new quadruped_msgs.msg.dds.QuadrupedStepMessagePubSubType(), data.getQuadrupedStepMessage());

   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage src, quadruped_msgs.msg.dds.QuadrupedTimedStepMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedTimedStepMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedTimedStepMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedTimedStepMessage src, quadruped_msgs.msg.dds.QuadrupedTimedStepMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedTimedStepMessagePubSubType newInstance()
   {
      return new QuadrupedTimedStepMessagePubSubType();
   }
}
