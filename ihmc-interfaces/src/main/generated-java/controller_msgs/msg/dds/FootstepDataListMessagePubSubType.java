package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepDataListMessage" defined in "FootstepDataListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepDataListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepDataListMessage_.idl instead.
*
*/
public class FootstepDataListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepDataListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepDataListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepDataListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepDataListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepDataListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepDataListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootstepDataList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getFootstepDataList().size() <= 50)
      cdr.write_type_e(data.getFootstepDataList());else
          throw new RuntimeException("footstep_data_list field exceeds the maximum length");

      cdr.write_type_9(data.getExecutionTiming());

      cdr.write_type_6(data.getDefaultSwingDuration());

      cdr.write_type_6(data.getDefaultTransferDuration());

      cdr.write_type_6(data.getFinalTransferDuration());

      cdr.write_type_6(data.getDefaultSwingDurationShiftFraction());

      cdr.write_type_6(data.getDefaultSwingSplitFraction());

      cdr.write_type_6(data.getDefaultTransferSplitFraction());

      cdr.write_type_6(data.getFinalTransferSplitFraction());

      cdr.write_type_7(data.getTrustHeightOfFootsteps());

      cdr.write_type_7(data.getAreFootstepsAdjustable());

      cdr.write_type_7(data.getOffsetFootstepsWithExecutionError());

      cdr.write_type_7(data.getOffsetFootstepsHeightWithExecutionError());

      controller_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
   }

   public static void read(controller_msgs.msg.dds.FootstepDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getFootstepDataList());	
      data.setExecutionTiming(cdr.read_type_9());
      	
      data.setDefaultSwingDuration(cdr.read_type_6());
      	
      data.setDefaultTransferDuration(cdr.read_type_6());
      	
      data.setFinalTransferDuration(cdr.read_type_6());
      	
      data.setDefaultSwingDurationShiftFraction(cdr.read_type_6());
      	
      data.setDefaultSwingSplitFraction(cdr.read_type_6());
      	
      data.setDefaultTransferSplitFraction(cdr.read_type_6());
      	
      data.setFinalTransferSplitFraction(cdr.read_type_6());
      	
      data.setTrustHeightOfFootsteps(cdr.read_type_7());
      	
      data.setAreFootstepsAdjustable(cdr.read_type_7());
      	
      data.setOffsetFootstepsWithExecutionError(cdr.read_type_7());
      	
      data.setOffsetFootstepsHeightWithExecutionError(cdr.read_type_7());
      	
      controller_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepDataListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("footstep_data_list", data.getFootstepDataList());
      ser.write_type_9("execution_timing", data.getExecutionTiming());
      ser.write_type_6("default_swing_duration", data.getDefaultSwingDuration());
      ser.write_type_6("default_transfer_duration", data.getDefaultTransferDuration());
      ser.write_type_6("final_transfer_duration", data.getFinalTransferDuration());
      ser.write_type_6("default_swing_duration_shift_fraction", data.getDefaultSwingDurationShiftFraction());
      ser.write_type_6("default_swing_split_fraction", data.getDefaultSwingSplitFraction());
      ser.write_type_6("default_transfer_split_fraction", data.getDefaultTransferSplitFraction());
      ser.write_type_6("final_transfer_split_fraction", data.getFinalTransferSplitFraction());
      ser.write_type_7("trust_height_of_footsteps", data.getTrustHeightOfFootsteps());
      ser.write_type_7("are_footsteps_adjustable", data.getAreFootstepsAdjustable());
      ser.write_type_7("offset_footsteps_with_execution_error", data.getOffsetFootstepsWithExecutionError());
      ser.write_type_7("offset_footsteps_height_with_execution_error", data.getOffsetFootstepsHeightWithExecutionError());
      ser.write_type_a("queueing_properties", new controller_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepDataListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("footstep_data_list", data.getFootstepDataList());
      data.setExecutionTiming(ser.read_type_9("execution_timing"));
      data.setDefaultSwingDuration(ser.read_type_6("default_swing_duration"));
      data.setDefaultTransferDuration(ser.read_type_6("default_transfer_duration"));
      data.setFinalTransferDuration(ser.read_type_6("final_transfer_duration"));
      data.setDefaultSwingDurationShiftFraction(ser.read_type_6("default_swing_duration_shift_fraction"));
      data.setDefaultSwingSplitFraction(ser.read_type_6("default_swing_split_fraction"));
      data.setDefaultTransferSplitFraction(ser.read_type_6("default_transfer_split_fraction"));
      data.setFinalTransferSplitFraction(ser.read_type_6("final_transfer_split_fraction"));
      data.setTrustHeightOfFootsteps(ser.read_type_7("trust_height_of_footsteps"));
      data.setAreFootstepsAdjustable(ser.read_type_7("are_footsteps_adjustable"));
      data.setOffsetFootstepsWithExecutionError(ser.read_type_7("offset_footsteps_with_execution_error"));
      data.setOffsetFootstepsHeightWithExecutionError(ser.read_type_7("offset_footsteps_height_with_execution_error"));
      ser.read_type_a("queueing_properties", new controller_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepDataListMessage src, controller_msgs.msg.dds.FootstepDataListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepDataListMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepDataListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepDataListMessage src, controller_msgs.msg.dds.FootstepDataListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepDataListMessagePubSubType newInstance()
   {
      return new FootstepDataListMessagePubSubType();
   }
}
