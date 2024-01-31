package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SE3TrajectoryMessage" defined in "SE3TrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SE3TrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SE3TrajectoryMessage_.idl instead.
*
*/
public class SE3TrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.SE3TrajectoryMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::SE3TrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9dd6ee74575933edd4c126b08f0bd65e0d3f2d7fe8823c1abf44de187a36dd2e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data) throws java.io.IOException
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
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.FrameInformationPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTaskspaceTrajectoryPoints().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getTaskspaceTrajectoryPoints().get(i0), current_alignment);}

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularSelectionMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearSelectionMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.FrameInformationPubSubType.getCdrSerializedSize(data.getFrameInformation(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularWeightMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearWeightMatrix(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getControlFramePose(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getTaskspaceTrajectoryPoints().size() <= 50)
      cdr.write_type_e(data.getTaskspaceTrajectoryPoints());else
          throw new RuntimeException("taskspace_trajectory_points field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getAngularSelectionMatrix(), cdr);
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getLinearSelectionMatrix(), cdr);
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.write(data.getFrameInformation(), cdr);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getAngularWeightMatrix(), cdr);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getLinearWeightMatrix(), cdr);
      cdr.write_type_7(data.getUseCustomControlFrame());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getControlFramePose(), cdr);
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getTaskspaceTrajectoryPoints());	
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getAngularSelectionMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getLinearSelectionMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.read(data.getFrameInformation(), cdr);	
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getAngularWeightMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getLinearWeightMatrix(), cdr);	
      data.setUseCustomControlFrame(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getControlFramePose(), cdr);	
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("taskspace_trajectory_points", data.getTaskspaceTrajectoryPoints());
      ser.write_type_a("angular_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());

      ser.write_type_a("linear_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());

      ser.write_type_a("frame_information", new ihmc_common_msgs.msg.dds.FrameInformationPubSubType(), data.getFrameInformation());

      ser.write_type_a("angular_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getAngularWeightMatrix());

      ser.write_type_a("linear_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getLinearWeightMatrix());

      ser.write_type_7("use_custom_control_frame", data.getUseCustomControlFrame());
      ser.write_type_a("control_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getControlFramePose());

      ser.write_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("taskspace_trajectory_points", data.getTaskspaceTrajectoryPoints());
      ser.read_type_a("angular_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());

      ser.read_type_a("linear_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());

      ser.read_type_a("frame_information", new ihmc_common_msgs.msg.dds.FrameInformationPubSubType(), data.getFrameInformation());

      ser.read_type_a("angular_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getAngularWeightMatrix());

      ser.read_type_a("linear_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getLinearWeightMatrix());

      data.setUseCustomControlFrame(ser.read_type_7("use_custom_control_frame"));
      ser.read_type_a("control_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getControlFramePose());

      ser.read_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage src, ihmc_common_msgs.msg.dds.SE3TrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.SE3TrajectoryMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.SE3TrajectoryMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.SE3TrajectoryMessage src, ihmc_common_msgs.msg.dds.SE3TrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SE3TrajectoryMessagePubSubType newInstance()
   {
      return new SE3TrajectoryMessagePubSubType();
   }
}
