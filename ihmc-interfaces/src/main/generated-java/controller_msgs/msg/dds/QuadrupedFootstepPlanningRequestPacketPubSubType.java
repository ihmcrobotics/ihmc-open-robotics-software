package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedFootstepPlanningRequestPacket" defined in "QuadrupedFootstepPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedFootstepPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedFootstepPlanningRequestPacket_.idl instead.
*
*/
public class QuadrupedFootstepPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedFootstepPlanningRequestPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBodyPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getBodyOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getGoalOrientationInWorld(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsListMessage(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getBodyPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getBodyOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getGoalOrientationInWorld(), cdr);
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsListMessage(), cdr);
      cdr.write_type_2(data.getPlannerRequestId());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getBodyPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getBodyOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getGoalOrientationInWorld(), cdr);	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsListMessage(), cdr);	
      data.setPlannerRequestId(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("body_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getBodyPositionInWorld());

      ser.write_type_a("body_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getBodyOrientationInWorld());

      ser.write_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.write_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      ser.write_type_a("planar_regions_list_message", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      ser.write_type_2("planner_request_id", data.getPlannerRequestId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("body_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getBodyPositionInWorld());

      ser.read_type_a("body_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getBodyOrientationInWorld());

      ser.read_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.read_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      ser.read_type_a("planar_regions_list_message", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      data.setPlannerRequestId(ser.read_type_2("planner_request_id"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket src, controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket createData()
   {
      return new controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket src, controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedFootstepPlanningRequestPacketPubSubType newInstance()
   {
      return new QuadrupedFootstepPlanningRequestPacketPubSubType();
   }
}
