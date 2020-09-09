package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TaskspaceTrajectoryStatusMessage" defined in "TaskspaceTrajectoryStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TaskspaceTrajectoryStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TaskspaceTrajectoryStatusMessage_.idl instead.
*
*/
public class TaskspaceTrajectoryStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::TaskspaceTrajectoryStatusMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getEndEffectorName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredEndEffectorPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredEndEffectorOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getActualEndEffectorPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getActualEndEffectorOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getEndEffectorName().length() <= 255)
      cdr.write_type_d(data.getEndEffectorName());else
          throw new RuntimeException("end_effector_name field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectoryExecutionStatus());

      cdr.write_type_6(data.getTimestamp());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredEndEffectorPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredEndEffectorOrientation(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getActualEndEffectorPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getActualEndEffectorOrientation(), cdr);
   }

   public static void read(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getEndEffectorName());	
      data.setTrajectoryExecutionStatus(cdr.read_type_9());
      	
      data.setTimestamp(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredEndEffectorPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredEndEffectorOrientation(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getActualEndEffectorPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getActualEndEffectorOrientation(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("end_effector_name", data.getEndEffectorName());
      ser.write_type_9("trajectory_execution_status", data.getTrajectoryExecutionStatus());
      ser.write_type_6("timestamp", data.getTimestamp());
      ser.write_type_a("desired_end_effector_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredEndEffectorPosition());

      ser.write_type_a("desired_end_effector_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredEndEffectorOrientation());

      ser.write_type_a("actual_end_effector_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualEndEffectorPosition());

      ser.write_type_a("actual_end_effector_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getActualEndEffectorOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("end_effector_name", data.getEndEffectorName());
      data.setTrajectoryExecutionStatus(ser.read_type_9("trajectory_execution_status"));
      data.setTimestamp(ser.read_type_6("timestamp"));
      ser.read_type_a("desired_end_effector_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredEndEffectorPosition());

      ser.read_type_a("desired_end_effector_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredEndEffectorOrientation());

      ser.read_type_a("actual_end_effector_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualEndEffectorPosition());

      ser.read_type_a("actual_end_effector_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getActualEndEffectorOrientation());

   }

   public static void staticCopy(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage src, controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage createData()
   {
      return new controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage src, controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TaskspaceTrajectoryStatusMessagePubSubType newInstance()
   {
      return new TaskspaceTrajectoryStatusMessagePubSubType();
   }
}
