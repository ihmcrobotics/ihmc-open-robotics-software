package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedFootstepStatusMessage" defined in "QuadrupedFootstepStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedFootstepStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedFootstepStatusMessage_.idl instead.
*
*/
public class QuadrupedFootstepStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedFootstepStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedFootstepStatusMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredTouchdownPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getActualTouchdownPositionInWorld(), current_alignment);

      current_alignment += controller_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getDesiredStepInterval(), current_alignment);

      current_alignment += controller_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getActualStepInterval(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFootstepStatus());

      cdr.write_type_2(data.getFootstepIndex());

      cdr.write_type_9(data.getRobotQuadrant());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredTouchdownPositionInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getActualTouchdownPositionInWorld(), cdr);
      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.write(data.getDesiredStepInterval(), cdr);
      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.write(data.getActualStepInterval(), cdr);
   }

   public static void read(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFootstepStatus(cdr.read_type_9());
      	
      data.setFootstepIndex(cdr.read_type_2());
      	
      data.setRobotQuadrant(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredTouchdownPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getActualTouchdownPositionInWorld(), cdr);	
      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.read(data.getDesiredStepInterval(), cdr);	
      controller_msgs.msg.dds.TimeIntervalMessagePubSubType.read(data.getActualStepInterval(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("footstep_status", data.getFootstepStatus());
      ser.write_type_2("footstep_index", data.getFootstepIndex());
      ser.write_type_9("robot_quadrant", data.getRobotQuadrant());
      ser.write_type_a("desired_touchdown_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredTouchdownPositionInWorld());

      ser.write_type_a("actual_touchdown_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualTouchdownPositionInWorld());

      ser.write_type_a("desired_step_interval", new controller_msgs.msg.dds.TimeIntervalMessagePubSubType(), data.getDesiredStepInterval());

      ser.write_type_a("actual_step_interval", new controller_msgs.msg.dds.TimeIntervalMessagePubSubType(), data.getActualStepInterval());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFootstepStatus(ser.read_type_9("footstep_status"));
      data.setFootstepIndex(ser.read_type_2("footstep_index"));
      data.setRobotQuadrant(ser.read_type_9("robot_quadrant"));
      ser.read_type_a("desired_touchdown_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredTouchdownPositionInWorld());

      ser.read_type_a("actual_touchdown_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualTouchdownPositionInWorld());

      ser.read_type_a("desired_step_interval", new controller_msgs.msg.dds.TimeIntervalMessagePubSubType(), data.getDesiredStepInterval());

      ser.read_type_a("actual_step_interval", new controller_msgs.msg.dds.TimeIntervalMessagePubSubType(), data.getActualStepInterval());

   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage src, controller_msgs.msg.dds.QuadrupedFootstepStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedFootstepStatusMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedFootstepStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedFootstepStatusMessage src, controller_msgs.msg.dds.QuadrupedFootstepStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedFootstepStatusMessagePubSubType newInstance()
   {
      return new QuadrupedFootstepStatusMessagePubSubType();
   }
}
