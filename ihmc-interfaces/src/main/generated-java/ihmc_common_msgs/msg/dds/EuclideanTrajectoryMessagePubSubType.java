package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EuclideanTrajectoryMessage" defined in "EuclideanTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EuclideanTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EuclideanTrajectoryMessage_.idl instead.
*
*/
public class EuclideanTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::EuclideanTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "32f6cd72887fe06094e7ecb23ac2bd5554adaff6f92612da380f4454bcc32bd1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data) throws java.io.IOException
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
          current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.FrameInformationPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTaskspaceTrajectoryPoints().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getTaskspaceTrajectoryPoints().get(i0), current_alignment);}

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getSelectionMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.FrameInformationPubSubType.getCdrSerializedSize(data.getFrameInformation(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getWeightMatrix(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getControlFramePose(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getTaskspaceTrajectoryPoints().size() <= 50)
      cdr.write_type_e(data.getTaskspaceTrajectoryPoints());else
          throw new RuntimeException("taskspace_trajectory_points field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getSelectionMatrix(), cdr);
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.write(data.getFrameInformation(), cdr);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getWeightMatrix(), cdr);
      cdr.write_type_7(data.getUseCustomControlFrame());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getControlFramePose(), cdr);
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getTaskspaceTrajectoryPoints());	
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getSelectionMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.read(data.getFrameInformation(), cdr);	
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getWeightMatrix(), cdr);	
      data.setUseCustomControlFrame(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getControlFramePose(), cdr);	
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("taskspace_trajectory_points", data.getTaskspaceTrajectoryPoints());
      ser.write_type_a("selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());

      ser.write_type_a("frame_information", new ihmc_common_msgs.msg.dds.FrameInformationPubSubType(), data.getFrameInformation());

      ser.write_type_a("weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeightMatrix());

      ser.write_type_7("use_custom_control_frame", data.getUseCustomControlFrame());
      ser.write_type_a("control_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getControlFramePose());

      ser.write_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("taskspace_trajectory_points", data.getTaskspaceTrajectoryPoints());
      ser.read_type_a("selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());

      ser.read_type_a("frame_information", new ihmc_common_msgs.msg.dds.FrameInformationPubSubType(), data.getFrameInformation());

      ser.read_type_a("weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeightMatrix());

      data.setUseCustomControlFrame(ser.read_type_7("use_custom_control_frame"));
      ser.read_type_a("control_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getControlFramePose());

      ser.read_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage src, ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage src, ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EuclideanTrajectoryMessagePubSubType newInstance()
   {
      return new EuclideanTrajectoryMessagePubSubType();
   }
}
