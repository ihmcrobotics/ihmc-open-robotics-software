package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanningRequestPacket" defined in "FootstepPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanningRequestPacket_.idl instead.
*
*/
public class FootstepPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlanningRequestPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlanningRequestPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b09bcb36200aa67c3cd7ef4cf6705ac474a4c17f878c7f8fb370c831be4fb73f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getStartLeftFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getStartRightFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getGoalLeftFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getGoalRightFootPose(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPathWaypoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getBodyPathWaypoints().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsListMessage(), current_alignment);

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getHeightMapMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getReferencePlan(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getStartLeftFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getStartRightFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getGoalLeftFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getGoalRightFootPose(), cdr);
      cdr.write_type_9(data.getRequestedInitialStanceSide());

      cdr.write_type_7(data.getSnapGoalSteps());

      cdr.write_type_7(data.getAbortIfGoalStepSnappingFails());

      cdr.write_type_7(data.getAbortIfBodyPathPlannerFails());

      cdr.write_type_7(data.getPlanBodyPath());

      cdr.write_type_7(data.getPlanFootsteps());

      cdr.write_type_7(data.getPerformAStarSearch());

      if(data.getBodyPathWaypoints().size() <= 50)
      cdr.write_type_e(data.getBodyPathWaypoints());else
          throw new RuntimeException("body_path_waypoints field exceeds the maximum length");

      cdr.write_type_6(data.getGoalDistanceProximity());

      cdr.write_type_6(data.getGoalYawProximity());

      cdr.write_type_6(data.getTimeout());

      cdr.write_type_2(data.getMaxIterations());

      cdr.write_type_6(data.getHorizonLength());

      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsListMessage(), cdr);
      perception_msgs.msg.dds.HeightMapMessagePubSubType.write(data.getHeightMapMessage(), cdr);
      cdr.write_type_7(data.getAssumeFlatGround());

      cdr.write_type_2(data.getPlannerRequestId());

      cdr.write_type_6(data.getStatusPublishPeriod());

      cdr.write_type_9(data.getRequestedSwingPlanner());

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getReferencePlan(), cdr);
      cdr.write_type_7(data.getGenerateLog());

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getStartLeftFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getStartRightFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getGoalLeftFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getGoalRightFootPose(), cdr);	
      data.setRequestedInitialStanceSide(cdr.read_type_9());
      	
      data.setSnapGoalSteps(cdr.read_type_7());
      	
      data.setAbortIfGoalStepSnappingFails(cdr.read_type_7());
      	
      data.setAbortIfBodyPathPlannerFails(cdr.read_type_7());
      	
      data.setPlanBodyPath(cdr.read_type_7());
      	
      data.setPlanFootsteps(cdr.read_type_7());
      	
      data.setPerformAStarSearch(cdr.read_type_7());
      	
      cdr.read_type_e(data.getBodyPathWaypoints());	
      data.setGoalDistanceProximity(cdr.read_type_6());
      	
      data.setGoalYawProximity(cdr.read_type_6());
      	
      data.setTimeout(cdr.read_type_6());
      	
      data.setMaxIterations(cdr.read_type_2());
      	
      data.setHorizonLength(cdr.read_type_6());
      	
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsListMessage(), cdr);	
      perception_msgs.msg.dds.HeightMapMessagePubSubType.read(data.getHeightMapMessage(), cdr);	
      data.setAssumeFlatGround(cdr.read_type_7());
      	
      data.setPlannerRequestId(cdr.read_type_2());
      	
      data.setStatusPublishPeriod(cdr.read_type_6());
      	
      data.setRequestedSwingPlanner(cdr.read_type_9());
      	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getReferencePlan(), cdr);	
      data.setGenerateLog(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("start_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getStartLeftFootPose());

      ser.write_type_a("start_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getStartRightFootPose());

      ser.write_type_a("goal_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalLeftFootPose());

      ser.write_type_a("goal_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalRightFootPose());

      ser.write_type_9("requested_initial_stance_side", data.getRequestedInitialStanceSide());
      ser.write_type_7("snap_goal_steps", data.getSnapGoalSteps());
      ser.write_type_7("abort_if_goal_step_snapping_fails", data.getAbortIfGoalStepSnappingFails());
      ser.write_type_7("abort_if_body_path_planner_fails", data.getAbortIfBodyPathPlannerFails());
      ser.write_type_7("plan_body_path", data.getPlanBodyPath());
      ser.write_type_7("plan_footsteps", data.getPlanFootsteps());
      ser.write_type_7("perform_a_star_search", data.getPerformAStarSearch());
      ser.write_type_e("body_path_waypoints", data.getBodyPathWaypoints());
      ser.write_type_6("goal_distance_proximity", data.getGoalDistanceProximity());
      ser.write_type_6("goal_yaw_proximity", data.getGoalYawProximity());
      ser.write_type_6("timeout", data.getTimeout());
      ser.write_type_2("max_iterations", data.getMaxIterations());
      ser.write_type_6("horizon_length", data.getHorizonLength());
      ser.write_type_a("planar_regions_list_message", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      ser.write_type_a("height_map_message", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMapMessage());

      ser.write_type_7("assume_flat_ground", data.getAssumeFlatGround());
      ser.write_type_2("planner_request_id", data.getPlannerRequestId());
      ser.write_type_6("status_publish_period", data.getStatusPublishPeriod());
      ser.write_type_9("requested_swing_planner", data.getRequestedSwingPlanner());
      ser.write_type_a("reference_plan", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getReferencePlan());

      ser.write_type_7("generate_log", data.getGenerateLog());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("start_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getStartLeftFootPose());

      ser.read_type_a("start_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getStartRightFootPose());

      ser.read_type_a("goal_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalLeftFootPose());

      ser.read_type_a("goal_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalRightFootPose());

      data.setRequestedInitialStanceSide(ser.read_type_9("requested_initial_stance_side"));
      data.setSnapGoalSteps(ser.read_type_7("snap_goal_steps"));
      data.setAbortIfGoalStepSnappingFails(ser.read_type_7("abort_if_goal_step_snapping_fails"));
      data.setAbortIfBodyPathPlannerFails(ser.read_type_7("abort_if_body_path_planner_fails"));
      data.setPlanBodyPath(ser.read_type_7("plan_body_path"));
      data.setPlanFootsteps(ser.read_type_7("plan_footsteps"));
      data.setPerformAStarSearch(ser.read_type_7("perform_a_star_search"));
      ser.read_type_e("body_path_waypoints", data.getBodyPathWaypoints());
      data.setGoalDistanceProximity(ser.read_type_6("goal_distance_proximity"));
      data.setGoalYawProximity(ser.read_type_6("goal_yaw_proximity"));
      data.setTimeout(ser.read_type_6("timeout"));
      data.setMaxIterations(ser.read_type_2("max_iterations"));
      data.setHorizonLength(ser.read_type_6("horizon_length"));
      ser.read_type_a("planar_regions_list_message", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      ser.read_type_a("height_map_message", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMapMessage());

      data.setAssumeFlatGround(ser.read_type_7("assume_flat_ground"));
      data.setPlannerRequestId(ser.read_type_2("planner_request_id"));
      data.setStatusPublishPeriod(ser.read_type_6("status_publish_period"));
      data.setRequestedSwingPlanner(ser.read_type_9("requested_swing_planner"));
      ser.read_type_a("reference_plan", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getReferencePlan());

      data.setGenerateLog(ser.read_type_7("generate_log"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket src, toolbox_msgs.msg.dds.FootstepPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlanningRequestPacket createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlanningRequestPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlanningRequestPacket src, toolbox_msgs.msg.dds.FootstepPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanningRequestPacketPubSubType newInstance()
   {
      return new FootstepPlanningRequestPacketPubSubType();
   }
}
