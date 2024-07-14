package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanActionDefinitionMessage" defined in "FootstepPlanActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanActionDefinitionMessage_.idl instead.
*
*/
public class FootstepPlanActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepPlanActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5e0ca5b87d2ca75a54a7c37483ac268b06d77b16d49bf6274d4b8b511a4fcac9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootsteps().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessagePubSubType.getCdrSerializedSize(data.getFootsteps().get(i0), current_alignment);}

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



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

      cdr.write_type_2(data.getExecutionMode());

      cdr.write_type_7(data.getIsManuallyPlaced());

      if(data.getFootsteps().size() <= 50)
      cdr.write_type_e(data.getFootsteps());else
          throw new RuntimeException("footsteps field exceeds the maximum length");

      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalStancePoint(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalFocalPoint(), cdr);
      cdr.write_type_6(data.getLeftGoalFootXToGizmo());

      cdr.write_type_6(data.getLeftGoalFootYToGizmo());

      cdr.write_type_6(data.getLeftGoalFootYawToGizmo());

      cdr.write_type_6(data.getRightGoalFootXToGizmo());

      cdr.write_type_6(data.getRightGoalFootYToGizmo());

      cdr.write_type_6(data.getRightGoalFootYawToGizmo());

      cdr.write_type_9(data.getPlannerInitialStanceSide());

      cdr.write_type_7(data.getPlannerUseTurnWalkTurn());

   }

   public static void read(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	
      data.setExecutionMode(cdr.read_type_2());
      	
      data.setIsManuallyPlaced(cdr.read_type_7());
      	
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
      	
      data.setPlannerUseTurnWalkTurn(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
      ser.write_type_2("execution_mode", data.getExecutionMode());
      ser.write_type_7("is_manually_placed", data.getIsManuallyPlaced());
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
      ser.write_type_7("planner_use_turn_walk_turn", data.getPlannerUseTurnWalkTurn());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
      data.setExecutionMode(ser.read_type_2("execution_mode"));
      data.setIsManuallyPlaced(ser.read_type_7("is_manually_placed"));
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
      data.setPlannerUseTurnWalkTurn(ser.read_type_7("planner_use_turn_walk_turn"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage src, behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage src, behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanActionDefinitionMessagePubSubType newInstance()
   {
      return new FootstepPlanActionDefinitionMessagePubSubType();
   }
}
