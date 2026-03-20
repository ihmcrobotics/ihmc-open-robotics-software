package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkActionDefinitionMessage" defined in "WalkActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkActionDefinitionMessage_.idl instead.
*
*/
public class WalkActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WalkActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WalkActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "72522f15f7d471fbd92e107da7332eefccf5ac82d5537e435fd6b260d55fc524";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WalkActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 40; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WalkActionFootstepDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWaypoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getWaypoints().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootsteps().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WalkActionFootstepDefinitionMessagePubSubType.getCdrSerializedSize(data.getFootsteps().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalStancePoint(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalFocalPoint(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType.getCdrSerializedSize(data.getPlannerParameters(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length: %d > %d".formatted(data.getParentFrameName().length(), 255));

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

      cdr.write_type_9(data.getExecutionMode());

      cdr.write_type_7(data.getIsManuallyPlaced());

      if(data.getWaypoints().size() <= 40)
      cdr.write_type_e(data.getWaypoints());else
          throw new RuntimeException("waypoints field exceeds the maximum length: %d > %d".formatted(data.getWaypoints().size(), 40));

      if(data.getFootsteps().size() <= 50)
      cdr.write_type_e(data.getFootsteps());else
          throw new RuntimeException("footsteps field exceeds the maximum length: %d > %d".formatted(data.getFootsteps().size(), 50));

      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalStancePoint(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalFocalPoint(), cdr);
      cdr.write_type_6(data.getLeftGoalFootXToGizmo());

      cdr.write_type_6(data.getLeftGoalFootYToGizmo());

      cdr.write_type_6(data.getLeftGoalFootYawToGizmo());

      cdr.write_type_6(data.getRightGoalFootXToGizmo());

      cdr.write_type_6(data.getRightGoalFootYToGizmo());

      cdr.write_type_6(data.getRightGoalFootYawToGizmo());

      cdr.write_type_9(data.getPlannerInitialStanceSide());

      cdr.write_type_9(data.getPlanner());

      cdr.write_type_7(data.getPlannerWalkWithGoalOrientation());

      cdr.write_type_7(data.getPlannerPlanWithBodyPath());

      ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType.write(data.getPlannerParameters(), cdr);
      cdr.write_type_7(data.getQuickWaypointOnly());

      cdr.write_type_7(data.getUseRrtPathPlanner());

      cdr.write_type_6(data.getQuickHipWidth());

      cdr.write_type_6(data.getQuickStepLength());

      cdr.write_type_6(data.getQuickNextPelvisYawLimit());

      cdr.write_type_6(data.getQuickInwardLimit());

      cdr.write_type_6(data.getQuickOutwardLimit());

      cdr.write_type_6(data.getQuickStepAngleLimit());

      cdr.write_type_6(data.getQuickSwingTimeDistanceLower());

      cdr.write_type_6(data.getQuickSwingTimeDistanceUpper());

      cdr.write_type_6(data.getQuickMinSwingTime());

      cdr.write_type_6(data.getQuickMaxSwingTime());

   }

   public static void read(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	
      data.setExecutionMode(cdr.read_type_9());
      	
      data.setIsManuallyPlaced(cdr.read_type_7());
      	
      cdr.read_type_e(data.getWaypoints());	
      cdr.read_type_e(data.getFootsteps());	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalStancePoint(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalFocalPoint(), cdr);	
      data.setLeftGoalFootXToGizmo(cdr.read_type_6());
      	
      data.setLeftGoalFootYToGizmo(cdr.read_type_6());
      	
      data.setLeftGoalFootYawToGizmo(cdr.read_type_6());
      	
      data.setRightGoalFootXToGizmo(cdr.read_type_6());
      	
      data.setRightGoalFootYToGizmo(cdr.read_type_6());
      	
      data.setRightGoalFootYawToGizmo(cdr.read_type_6());
      	
      data.setPlannerInitialStanceSide(cdr.read_type_9());
      	
      data.setPlanner(cdr.read_type_9());
      	
      data.setPlannerWalkWithGoalOrientation(cdr.read_type_7());
      	
      data.setPlannerPlanWithBodyPath(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType.read(data.getPlannerParameters(), cdr);	
      data.setQuickWaypointOnly(cdr.read_type_7());
      	
      data.setUseRrtPathPlanner(cdr.read_type_7());
      	
      data.setQuickHipWidth(cdr.read_type_6());
      	
      data.setQuickStepLength(cdr.read_type_6());
      	
      data.setQuickNextPelvisYawLimit(cdr.read_type_6());
      	
      data.setQuickInwardLimit(cdr.read_type_6());
      	
      data.setQuickOutwardLimit(cdr.read_type_6());
      	
      data.setQuickStepAngleLimit(cdr.read_type_6());
      	
      data.setQuickSwingTimeDistanceLower(cdr.read_type_6());
      	
      data.setQuickSwingTimeDistanceUpper(cdr.read_type_6());
      	
      data.setQuickMinSwingTime(cdr.read_type_6());
      	
      data.setQuickMaxSwingTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
      ser.write_type_9("execution_mode", data.getExecutionMode());
      ser.write_type_7("is_manually_placed", data.getIsManuallyPlaced());
      ser.write_type_e("waypoints", data.getWaypoints());
      ser.write_type_e("footsteps", data.getFootsteps());
      ser.write_type_a("goal_stance_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalStancePoint());

      ser.write_type_a("goal_focal_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalFocalPoint());

      ser.write_type_6("left_goal_foot_x_to_gizmo", data.getLeftGoalFootXToGizmo());
      ser.write_type_6("left_goal_foot_y_to_gizmo", data.getLeftGoalFootYToGizmo());
      ser.write_type_6("left_goal_foot_yaw_to_gizmo", data.getLeftGoalFootYawToGizmo());
      ser.write_type_6("right_goal_foot_x_to_gizmo", data.getRightGoalFootXToGizmo());
      ser.write_type_6("right_goal_foot_y_to_gizmo", data.getRightGoalFootYToGizmo());
      ser.write_type_6("right_goal_foot_yaw_to_gizmo", data.getRightGoalFootYawToGizmo());
      ser.write_type_9("planner_initial_stance_side", data.getPlannerInitialStanceSide());
      ser.write_type_9("planner", data.getPlanner());
      ser.write_type_7("planner_walk_with_goal_orientation", data.getPlannerWalkWithGoalOrientation());
      ser.write_type_7("planner_plan_with_body_path", data.getPlannerPlanWithBodyPath());
      ser.write_type_a("planner_parameters", new ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType(), data.getPlannerParameters());

      ser.write_type_7("quick_waypoint_only", data.getQuickWaypointOnly());
      ser.write_type_7("use_rrt_path_planner", data.getUseRrtPathPlanner());
      ser.write_type_6("quick_hip_width", data.getQuickHipWidth());
      ser.write_type_6("quick_step_length", data.getQuickStepLength());
      ser.write_type_6("quick_next_pelvis_yaw_limit", data.getQuickNextPelvisYawLimit());
      ser.write_type_6("quick_inward_limit", data.getQuickInwardLimit());
      ser.write_type_6("quick_outward_limit", data.getQuickOutwardLimit());
      ser.write_type_6("quick_step_angle_limit", data.getQuickStepAngleLimit());
      ser.write_type_6("quick_swing_time_distance_lower", data.getQuickSwingTimeDistanceLower());
      ser.write_type_6("quick_swing_time_distance_upper", data.getQuickSwingTimeDistanceUpper());
      ser.write_type_6("quick_min_swing_time", data.getQuickMinSwingTime());
      ser.write_type_6("quick_max_swing_time", data.getQuickMaxSwingTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WalkActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
      data.setExecutionMode(ser.read_type_9("execution_mode"));
      data.setIsManuallyPlaced(ser.read_type_7("is_manually_placed"));
      ser.read_type_e("waypoints", data.getWaypoints());
      ser.read_type_e("footsteps", data.getFootsteps());
      ser.read_type_a("goal_stance_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalStancePoint());

      ser.read_type_a("goal_focal_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalFocalPoint());

      data.setLeftGoalFootXToGizmo(ser.read_type_6("left_goal_foot_x_to_gizmo"));
      data.setLeftGoalFootYToGizmo(ser.read_type_6("left_goal_foot_y_to_gizmo"));
      data.setLeftGoalFootYawToGizmo(ser.read_type_6("left_goal_foot_yaw_to_gizmo"));
      data.setRightGoalFootXToGizmo(ser.read_type_6("right_goal_foot_x_to_gizmo"));
      data.setRightGoalFootYToGizmo(ser.read_type_6("right_goal_foot_y_to_gizmo"));
      data.setRightGoalFootYawToGizmo(ser.read_type_6("right_goal_foot_yaw_to_gizmo"));
      data.setPlannerInitialStanceSide(ser.read_type_9("planner_initial_stance_side"));
      data.setPlanner(ser.read_type_9("planner"));
      data.setPlannerWalkWithGoalOrientation(ser.read_type_7("planner_walk_with_goal_orientation"));
      data.setPlannerPlanWithBodyPath(ser.read_type_7("planner_plan_with_body_path"));
      ser.read_type_a("planner_parameters", new ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType(), data.getPlannerParameters());

      data.setQuickWaypointOnly(ser.read_type_7("quick_waypoint_only"));
      data.setUseRrtPathPlanner(ser.read_type_7("use_rrt_path_planner"));
      data.setQuickHipWidth(ser.read_type_6("quick_hip_width"));
      data.setQuickStepLength(ser.read_type_6("quick_step_length"));
      data.setQuickNextPelvisYawLimit(ser.read_type_6("quick_next_pelvis_yaw_limit"));
      data.setQuickInwardLimit(ser.read_type_6("quick_inward_limit"));
      data.setQuickOutwardLimit(ser.read_type_6("quick_outward_limit"));
      data.setQuickStepAngleLimit(ser.read_type_6("quick_step_angle_limit"));
      data.setQuickSwingTimeDistanceLower(ser.read_type_6("quick_swing_time_distance_lower"));
      data.setQuickSwingTimeDistanceUpper(ser.read_type_6("quick_swing_time_distance_upper"));
      data.setQuickMinSwingTime(ser.read_type_6("quick_min_swing_time"));
      data.setQuickMaxSwingTime(ser.read_type_6("quick_max_swing_time"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.WalkActionDefinitionMessage src, behavior_msgs.msg.dds.WalkActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WalkActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.WalkActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WalkActionDefinitionMessage src, behavior_msgs.msg.dds.WalkActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkActionDefinitionMessagePubSubType newInstance()
   {
      return new WalkActionDefinitionMessagePubSubType();
   }
}
