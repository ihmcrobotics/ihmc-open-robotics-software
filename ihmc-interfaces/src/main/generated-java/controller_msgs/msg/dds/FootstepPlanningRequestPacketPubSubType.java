package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanningRequestPacket" defined in "FootstepPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanningRequestPacket_.idl instead.
*
*/
public class FootstepPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlanningRequestPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlanningRequestPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getStanceFootPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getStanceFootOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getGoalOrientationInWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsListMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getInitialStanceRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getStanceFootPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getStanceFootOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getGoalOrientationInWorld(), cdr);
      cdr.write_type_9(data.getRequestedFootstepPlannerType());

      cdr.write_type_6(data.getGoalDistanceProximity());

      cdr.write_type_6(data.getGoalYawProximity());

      cdr.write_type_6(data.getTimeout());

      cdr.write_type_6(data.getHorizonLength());

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsListMessage(), cdr);
      cdr.write_type_7(data.getAssumeFlatGround());

      cdr.write_type_2(data.getPlannerRequestId());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setInitialStanceRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getStanceFootPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getStanceFootOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getGoalOrientationInWorld(), cdr);	
      data.setRequestedFootstepPlannerType(cdr.read_type_9());
      	
      data.setGoalDistanceProximity(cdr.read_type_6());
      	
      data.setGoalYawProximity(cdr.read_type_6());
      	
      data.setTimeout(cdr.read_type_6());
      	
      data.setHorizonLength(cdr.read_type_6());
      	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsListMessage(), cdr);	
      data.setAssumeFlatGround(cdr.read_type_7());
      	
      data.setPlannerRequestId(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("initial_stance_robot_side", data.getInitialStanceRobotSide());
      ser.write_type_a("stance_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getStanceFootPositionInWorld());

      ser.write_type_a("stance_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getStanceFootOrientationInWorld());

      ser.write_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.write_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      ser.write_type_9("requested_footstep_planner_type", data.getRequestedFootstepPlannerType());
      ser.write_type_6("goal_distance_proximity", data.getGoalDistanceProximity());
      ser.write_type_6("goal_yaw_proximity", data.getGoalYawProximity());
      ser.write_type_6("timeout", data.getTimeout());
      ser.write_type_6("horizon_length", data.getHorizonLength());
      ser.write_type_a("planar_regions_list_message", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      ser.write_type_7("assume_flat_ground", data.getAssumeFlatGround());
      ser.write_type_2("planner_request_id", data.getPlannerRequestId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlanningRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setInitialStanceRobotSide(ser.read_type_9("initial_stance_robot_side"));
      ser.read_type_a("stance_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getStanceFootPositionInWorld());

      ser.read_type_a("stance_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getStanceFootOrientationInWorld());

      ser.read_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.read_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      data.setRequestedFootstepPlannerType(ser.read_type_9("requested_footstep_planner_type"));
      data.setGoalDistanceProximity(ser.read_type_6("goal_distance_proximity"));
      data.setGoalYawProximity(ser.read_type_6("goal_yaw_proximity"));
      data.setTimeout(ser.read_type_6("timeout"));
      data.setHorizonLength(ser.read_type_6("horizon_length"));
      ser.read_type_a("planar_regions_list_message", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      data.setAssumeFlatGround(ser.read_type_7("assume_flat_ground"));
      data.setPlannerRequestId(ser.read_type_2("planner_request_id"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlanningRequestPacket src, controller_msgs.msg.dds.FootstepPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlanningRequestPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPlanningRequestPacket();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlanningRequestPacket src, controller_msgs.msg.dds.FootstepPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanningRequestPacketPubSubType newInstance()
   {
      return new FootstepPlanningRequestPacketPubSubType();
   }
}
