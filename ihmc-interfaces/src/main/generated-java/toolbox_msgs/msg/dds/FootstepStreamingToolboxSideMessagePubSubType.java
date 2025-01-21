package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepStreamingToolboxSideMessage" defined in "FootstepStreamingToolboxSideMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepStreamingToolboxSideMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepStreamingToolboxSideMessage_.idl instead.
*
*/
public class FootstepStreamingToolboxSideMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepStreamingToolboxSideMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0896015bff5c97d2fe0c152d64171a9d547f3fe050438757f22f1f8f8a6f34b3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRobotFootPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRobotFootOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCurrentPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getCurrentOrientationInWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getCurrentLinearVelocityInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getCurrentAngularVelocityInWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_9(data.getSide());

      cdr.write_type_11(data.getTimestamp());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getRobotFootPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRobotFootOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getCurrentPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getCurrentOrientationInWorld(), cdr);
      cdr.write_type_7(data.getHasCurrentVelocity());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getCurrentLinearVelocityInWorld(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getCurrentAngularVelocityInWorld(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setSide(cdr.read_type_9());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRobotFootPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRobotFootOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getCurrentPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getCurrentOrientationInWorld(), cdr);	
      data.setHasCurrentVelocity(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getCurrentLinearVelocityInWorld(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getCurrentAngularVelocityInWorld(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_9("side", data.getSide());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_a("robot_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getRobotFootPositionInWorld());

      ser.write_type_a("robot_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRobotFootOrientationInWorld());

      ser.write_type_a("current_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getCurrentPositionInWorld());

      ser.write_type_a("current_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getCurrentOrientationInWorld());

      ser.write_type_7("has_current_velocity", data.getHasCurrentVelocity());
      ser.write_type_a("current_linear_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getCurrentLinearVelocityInWorld());

      ser.write_type_a("current_angular_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getCurrentAngularVelocityInWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setSide(ser.read_type_9("side"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      ser.read_type_a("robot_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getRobotFootPositionInWorld());

      ser.read_type_a("robot_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRobotFootOrientationInWorld());

      ser.read_type_a("current_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getCurrentPositionInWorld());

      ser.read_type_a("current_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getCurrentOrientationInWorld());

      data.setHasCurrentVelocity(ser.read_type_7("has_current_velocity"));
      ser.read_type_a("current_linear_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getCurrentLinearVelocityInWorld());

      ser.read_type_a("current_angular_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getCurrentAngularVelocityInWorld());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage src, toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage src, toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepStreamingToolboxSideMessagePubSubType newInstance()
   {
      return new FootstepStreamingToolboxSideMessagePubSubType();
   }
}
