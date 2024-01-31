package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepQueueStatusMessage" defined in "FootstepQueueStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepQueueStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepQueueStatusMessage_.idl instead.
*
*/
public class FootstepQueueStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepQueueStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepQueueStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "81d01e161604e18e5f4e3fec9f95d2d64e0ea7ab76379fcaa554a1e289816d8c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepQueueStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QueuedFootstepStatusMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepQueueStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getQueuedFootstepList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QueuedFootstepStatusMessagePubSubType.getCdrSerializedSize(data.getQueuedFootstepList().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIsFirstStepInSwing());

      cdr.write_type_6(data.getTimeInSupportSequence());

      if(data.getQueuedFootstepList().size() <= 50)
      cdr.write_type_e(data.getQueuedFootstepList());else
          throw new RuntimeException("queued_footstep_list field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIsFirstStepInSwing(cdr.read_type_7());
      	
      data.setTimeInSupportSequence(cdr.read_type_6());
      	
      cdr.read_type_e(data.getQueuedFootstepList());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("is_first_step_in_swing", data.getIsFirstStepInSwing());
      ser.write_type_6("time_in_support_sequence", data.getTimeInSupportSequence());
      ser.write_type_e("queued_footstep_list", data.getQueuedFootstepList());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepQueueStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIsFirstStepInSwing(ser.read_type_7("is_first_step_in_swing"));
      data.setTimeInSupportSequence(ser.read_type_6("time_in_support_sequence"));
      ser.read_type_e("queued_footstep_list", data.getQueuedFootstepList());
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepQueueStatusMessage src, controller_msgs.msg.dds.FootstepQueueStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepQueueStatusMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepQueueStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepQueueStatusMessage src, controller_msgs.msg.dds.FootstepQueueStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepQueueStatusMessagePubSubType newInstance()
   {
      return new FootstepQueueStatusMessagePubSubType();
   }
}
