package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepStreamingToolboxOutputStatus" defined in "FootstepStreamingToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepStreamingToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepStreamingToolboxOutputStatus_.idl instead.
*
*/
public class FootstepStreamingToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepStreamingToolboxOutputStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7837d499956970d7c7011c4d1073d7b81fc9f1eb2bc0e33aec3cc6aed4e1729c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredFootPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredFootOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredFootPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredFootOrientation(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredFootPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredFootOrientation(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("desired_foot_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredFootPosition());

      ser.write_type_a("desired_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredFootOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("desired_foot_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredFootPosition());

      ser.read_type_a("desired_foot_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredFootOrientation());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus src, toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus createData()
   {
      return new toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus src, toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepStreamingToolboxOutputStatusPubSubType newInstance()
   {
      return new FootstepStreamingToolboxOutputStatusPubSubType();
   }
}
