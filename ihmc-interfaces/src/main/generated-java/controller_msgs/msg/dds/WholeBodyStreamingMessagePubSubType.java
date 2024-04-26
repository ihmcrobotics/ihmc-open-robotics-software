package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyStreamingMessage" defined in "WholeBodyStreamingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyStreamingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyStreamingMessage_.idl instead.
*
*/
public class WholeBodyStreamingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyStreamingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyStreamingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9cefce68b58b7434f96f5fba2cff80a26d4ebfcbb0effeb059052069398dc61a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyStreamingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyStreamingMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyStreamingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyStreamingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.getCdrSerializedSize(data.getLeftHandStreamingMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.getCdrSerializedSize(data.getRightHandStreamingMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.getCdrSerializedSize(data.getLeftArmStreamingMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.getCdrSerializedSize(data.getRightArmStreamingMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType.getCdrSerializedSize(data.getChestStreamingMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.getCdrSerializedSize(data.getPelvisStreamingMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.getCdrSerializedSize(data.getNeckStreamingMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_5(data.getStreamIntegrationDuration());

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_7(data.getHasLeftHandStreamingMessage());

      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.write(data.getLeftHandStreamingMessage(), cdr);
      cdr.write_type_7(data.getHasRightHandStreamingMessage());

      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.write(data.getRightHandStreamingMessage(), cdr);
      cdr.write_type_7(data.getHasLeftArmStreamingMessage());

      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.write(data.getLeftArmStreamingMessage(), cdr);
      cdr.write_type_7(data.getHasRightArmStreamingMessage());

      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.write(data.getRightArmStreamingMessage(), cdr);
      cdr.write_type_7(data.getHasChestStreamingMessage());

      ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType.write(data.getChestStreamingMessage(), cdr);
      cdr.write_type_7(data.getHasPelvisStreamingMessage());

      cdr.write_type_7(data.getEnableUserPelvisControl());

      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.write(data.getPelvisStreamingMessage(), cdr);
      cdr.write_type_7(data.getHasNeckStreamingMessage());

      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.write(data.getNeckStreamingMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WholeBodyStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setStreamIntegrationDuration(cdr.read_type_5());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setHasLeftHandStreamingMessage(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.read(data.getLeftHandStreamingMessage(), cdr);	
      data.setHasRightHandStreamingMessage(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.read(data.getRightHandStreamingMessage(), cdr);	
      data.setHasLeftArmStreamingMessage(cdr.read_type_7());
      	
      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.read(data.getLeftArmStreamingMessage(), cdr);	
      data.setHasRightArmStreamingMessage(cdr.read_type_7());
      	
      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.read(data.getRightArmStreamingMessage(), cdr);	
      data.setHasChestStreamingMessage(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType.read(data.getChestStreamingMessage(), cdr);	
      data.setHasPelvisStreamingMessage(cdr.read_type_7());
      	
      data.setEnableUserPelvisControl(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType.read(data.getPelvisStreamingMessage(), cdr);	
      data.setHasNeckStreamingMessage(cdr.read_type_7());
      	
      controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType.read(data.getNeckStreamingMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyStreamingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_5("stream_integration_duration", data.getStreamIntegrationDuration());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_7("has_left_hand_streaming_message", data.getHasLeftHandStreamingMessage());
      ser.write_type_a("left_hand_streaming_message", new ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType(), data.getLeftHandStreamingMessage());

      ser.write_type_7("has_right_hand_streaming_message", data.getHasRightHandStreamingMessage());
      ser.write_type_a("right_hand_streaming_message", new ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType(), data.getRightHandStreamingMessage());

      ser.write_type_7("has_left_arm_streaming_message", data.getHasLeftArmStreamingMessage());
      ser.write_type_a("left_arm_streaming_message", new controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType(), data.getLeftArmStreamingMessage());

      ser.write_type_7("has_right_arm_streaming_message", data.getHasRightArmStreamingMessage());
      ser.write_type_a("right_arm_streaming_message", new controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType(), data.getRightArmStreamingMessage());

      ser.write_type_7("has_chest_streaming_message", data.getHasChestStreamingMessage());
      ser.write_type_a("chest_streaming_message", new ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType(), data.getChestStreamingMessage());

      ser.write_type_7("has_pelvis_streaming_message", data.getHasPelvisStreamingMessage());
      ser.write_type_7("enable_user_pelvis_control", data.getEnableUserPelvisControl());
      ser.write_type_a("pelvis_streaming_message", new ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType(), data.getPelvisStreamingMessage());

      ser.write_type_7("has_neck_streaming_message", data.getHasNeckStreamingMessage());
      ser.write_type_a("neck_streaming_message", new controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType(), data.getNeckStreamingMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyStreamingMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setStreamIntegrationDuration(ser.read_type_5("stream_integration_duration"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setHasLeftHandStreamingMessage(ser.read_type_7("has_left_hand_streaming_message"));
      ser.read_type_a("left_hand_streaming_message", new ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType(), data.getLeftHandStreamingMessage());

      data.setHasRightHandStreamingMessage(ser.read_type_7("has_right_hand_streaming_message"));
      ser.read_type_a("right_hand_streaming_message", new ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType(), data.getRightHandStreamingMessage());

      data.setHasLeftArmStreamingMessage(ser.read_type_7("has_left_arm_streaming_message"));
      ser.read_type_a("left_arm_streaming_message", new controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType(), data.getLeftArmStreamingMessage());

      data.setHasRightArmStreamingMessage(ser.read_type_7("has_right_arm_streaming_message"));
      ser.read_type_a("right_arm_streaming_message", new controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType(), data.getRightArmStreamingMessage());

      data.setHasChestStreamingMessage(ser.read_type_7("has_chest_streaming_message"));
      ser.read_type_a("chest_streaming_message", new ihmc_common_msgs.msg.dds.SO3StreamingMessagePubSubType(), data.getChestStreamingMessage());

      data.setHasPelvisStreamingMessage(ser.read_type_7("has_pelvis_streaming_message"));
      data.setEnableUserPelvisControl(ser.read_type_7("enable_user_pelvis_control"));
      ser.read_type_a("pelvis_streaming_message", new ihmc_common_msgs.msg.dds.SE3StreamingMessagePubSubType(), data.getPelvisStreamingMessage());

      data.setHasNeckStreamingMessage(ser.read_type_7("has_neck_streaming_message"));
      ser.read_type_a("neck_streaming_message", new controller_msgs.msg.dds.JointspaceStreamingMessagePubSubType(), data.getNeckStreamingMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyStreamingMessage src, controller_msgs.msg.dds.WholeBodyStreamingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyStreamingMessage createData()
   {
      return new controller_msgs.msg.dds.WholeBodyStreamingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyStreamingMessage src, controller_msgs.msg.dds.WholeBodyStreamingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyStreamingMessagePubSubType newInstance()
   {
      return new WholeBodyStreamingMessagePubSubType();
   }
}
