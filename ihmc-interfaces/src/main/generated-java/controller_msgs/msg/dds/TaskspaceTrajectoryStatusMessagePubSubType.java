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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getBodyName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredBodyPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredFootOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getActualBodyPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getActualBodyOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getBodyName().length() <= 255)
      cdr.write_type_d(data.getBodyName());else
          throw new RuntimeException("body_name field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectoryStatus());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredBodyPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredFootOrientation(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getActualBodyPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getActualBodyOrientation(), cdr);
   }

   public static void read(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getBodyName());	
      data.setTrajectoryStatus(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredBodyPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredFootOrientation(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getActualBodyPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getActualBodyOrientation(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("body_name", data.getBodyName());
      ser.write_type_9("trajectory_status", data.getTrajectoryStatus());
      ser.write_type_a("desired_body_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredBodyPosition());

      ser.write_type_a("desired_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredFootOrientation());

      ser.write_type_a("actual_body_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualBodyPosition());

      ser.write_type_a("actual_body_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getActualBodyOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("body_name", data.getBodyName());
      data.setTrajectoryStatus(ser.read_type_9("trajectory_status"));
      ser.read_type_a("desired_body_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredBodyPosition());

      ser.read_type_a("desired_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredFootOrientation());

      ser.read_type_a("actual_body_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getActualBodyPosition());

      ser.read_type_a("actual_body_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getActualBodyOrientation());

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
