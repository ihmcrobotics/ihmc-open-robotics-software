package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepNodeDataMessage" defined in "FootstepNodeDataMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepNodeDataMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepNodeDataMessage_.idl instead.
*
*/
public class FootstepNodeDataMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepNodeDataMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepNodeDataMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepNodeDataMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepNodeDataMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepNodeDataMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepNodeDataMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSnapTranslation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getSnapRotation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepNodeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_2(data.getXIndex());

      cdr.write_type_2(data.getYIndex());

      cdr.write_type_2(data.getYawIndex());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getSnapTranslation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getSnapRotation(), cdr);
      cdr.write_type_2(data.getParentNodeId());

      cdr.write_type_9(data.getBipedalFootstepPlannerNodeRejectionReason());

   }

   public static void read(controller_msgs.msg.dds.FootstepNodeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setXIndex(cdr.read_type_2());
      	
      data.setYIndex(cdr.read_type_2());
      	
      data.setYawIndex(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getSnapTranslation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getSnapRotation(), cdr);	
      data.setParentNodeId(cdr.read_type_2());
      	
      data.setBipedalFootstepPlannerNodeRejectionReason(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepNodeDataMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_2("x_index", data.getXIndex());
      ser.write_type_2("y_index", data.getYIndex());
      ser.write_type_2("yaw_index", data.getYawIndex());
      ser.write_type_a("snap_translation", new geometry_msgs.msg.dds.PointPubSubType(), data.getSnapTranslation());

      ser.write_type_a("snap_rotation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSnapRotation());

      ser.write_type_2("parent_node_id", data.getParentNodeId());
      ser.write_type_9("bipedal_footstep_planner_node_rejection_reason", data.getBipedalFootstepPlannerNodeRejectionReason());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepNodeDataMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setXIndex(ser.read_type_2("x_index"));
      data.setYIndex(ser.read_type_2("y_index"));
      data.setYawIndex(ser.read_type_2("yaw_index"));
      ser.read_type_a("snap_translation", new geometry_msgs.msg.dds.PointPubSubType(), data.getSnapTranslation());

      ser.read_type_a("snap_rotation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSnapRotation());

      data.setParentNodeId(ser.read_type_2("parent_node_id"));
      data.setBipedalFootstepPlannerNodeRejectionReason(ser.read_type_9("bipedal_footstep_planner_node_rejection_reason"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepNodeDataMessage src, controller_msgs.msg.dds.FootstepNodeDataMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepNodeDataMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepNodeDataMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepNodeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepNodeDataMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepNodeDataMessage src, controller_msgs.msg.dds.FootstepNodeDataMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepNodeDataMessagePubSubType newInstance()
   {
      return new FootstepNodeDataMessagePubSubType();
   }
}
