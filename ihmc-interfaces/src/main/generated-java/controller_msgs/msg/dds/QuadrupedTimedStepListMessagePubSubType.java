package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedTimedStepListMessage" defined in "QuadrupedTimedStepListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedTimedStepListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedTimedStepListMessage_.idl instead.
*
*/
public class QuadrupedTimedStepListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedTimedStepListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedTimedStepListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedTimedStepListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QuadrupedTimedStepMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getQuadrupedStepList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QuadrupedTimedStepMessagePubSubType.getCdrSerializedSize(data.getQuadrupedStepList().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getQuadrupedStepList().size() <= 100)
      cdr.write_type_e(data.getQuadrupedStepList());else
          throw new RuntimeException("quadruped_step_list field exceeds the maximum length");

      cdr.write_type_7(data.getIsExpressedInAbsoluteTime());

      controller_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
      cdr.write_type_7(data.getAreStepsAdjustable());

      cdr.write_type_7(data.getOffsetStepsHeightWithExecutionError());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getQuadrupedStepList());	
      data.setIsExpressedInAbsoluteTime(cdr.read_type_7());
      	
      controller_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	
      data.setAreStepsAdjustable(cdr.read_type_7());
      	
      data.setOffsetStepsHeightWithExecutionError(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("quadruped_step_list", data.getQuadrupedStepList());
      ser.write_type_7("is_expressed_in_absolute_time", data.getIsExpressedInAbsoluteTime());
      ser.write_type_a("queueing_properties", new controller_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

      ser.write_type_7("are_steps_adjustable", data.getAreStepsAdjustable());
      ser.write_type_7("offset_steps_height_with_execution_error", data.getOffsetStepsHeightWithExecutionError());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedTimedStepListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("quadruped_step_list", data.getQuadrupedStepList());
      data.setIsExpressedInAbsoluteTime(ser.read_type_7("is_expressed_in_absolute_time"));
      ser.read_type_a("queueing_properties", new controller_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

      data.setAreStepsAdjustable(ser.read_type_7("are_steps_adjustable"));
      data.setOffsetStepsHeightWithExecutionError(ser.read_type_7("offset_steps_height_with_execution_error"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedTimedStepListMessage src, controller_msgs.msg.dds.QuadrupedTimedStepListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedTimedStepListMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedTimedStepListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedTimedStepListMessage src, controller_msgs.msg.dds.QuadrupedTimedStepListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedTimedStepListMessagePubSubType newInstance()
   {
      return new QuadrupedTimedStepListMessagePubSubType();
   }
}
