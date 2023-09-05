package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepStatusMessage" defined in "FootstepStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepStatusMessage_.idl instead.
*
*/
public class FootstepStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "15eaba202ed9261a5fdf5e1e01a6a248719bca36a9ef63bab17cd7e69748df79";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredFootPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredFootOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getActualFootPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getActualFootOrientationInWorld(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFootstepStatus());

      cdr.write_type_2(data.getFootstepIndex());

      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredFootPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredFootOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getActualFootPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getActualFootOrientationInWorld(), cdr);
      cdr.write_type_6(data.getSwingDuration());

   }

   public static void read(controller_msgs.msg.dds.FootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFootstepStatus(cdr.read_type_9());
      	
      data.setFootstepIndex(cdr.read_type_2());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredFootPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredFootOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getActualFootPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getActualFootOrientationInWorld(), cdr);	
      data.setSwingDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("footstep_status", data.getFootstepStatus());
      ser.write_type_2("footstep_index", data.getFootstepIndex());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("desired_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredFootPositionInWorld());

      ser.write_type_a("desired_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredFootOrientationInWorld());

      ser.write_type_a("actual_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualFootPositionInWorld());

      ser.write_type_a("actual_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getActualFootOrientationInWorld());

      ser.write_type_6("swing_duration", data.getSwingDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFootstepStatus(ser.read_type_9("footstep_status"));
      data.setFootstepIndex(ser.read_type_2("footstep_index"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("desired_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredFootPositionInWorld());

      ser.read_type_a("desired_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredFootOrientationInWorld());

      ser.read_type_a("actual_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualFootPositionInWorld());

      ser.read_type_a("actual_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getActualFootOrientationInWorld());

      data.setSwingDuration(ser.read_type_6("swing_duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepStatusMessage src, controller_msgs.msg.dds.FootstepStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepStatusMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepStatusMessage src, controller_msgs.msg.dds.FootstepStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepStatusMessagePubSubType newInstance()
   {
      return new FootstepStatusMessagePubSubType();
   }
}
