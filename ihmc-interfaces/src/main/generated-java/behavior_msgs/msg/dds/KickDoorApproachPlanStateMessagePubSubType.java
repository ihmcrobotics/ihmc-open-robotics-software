package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KickDoorApproachPlanStateMessage" defined in "KickDoorApproachPlanStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KickDoorApproachPlanStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KickDoorApproachPlanStateMessage_.idl instead.
*
*/
public class KickDoorApproachPlanStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::KickDoorApproachPlanStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "02cbee0a296afe41ffd8e462465f1c0a8c85d3293d8ba89f7a7d4d59ba918691";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootsteps().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType.getCdrSerializedSize(data.getFootsteps().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getLeftFootGoalPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getRightFootGoalPose(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDesiredLeftFootsteps().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getDesiredLeftFootsteps().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDesiredRightFootsteps().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getDesiredRightFootsteps().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCurrentLeftFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCurrentRightFootPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getFootsteps().size() <= 100)
      cdr.write_type_e(data.getFootsteps());else
          throw new RuntimeException("footsteps field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getLeftFootGoalPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getRightFootGoalPose(), cdr);
      cdr.write_type_9(data.getExecutionState());

      cdr.write_type_3(data.getTotalNumberOfFootsteps());

      cdr.write_type_3(data.getNumberOfIncompleteFootsteps());

      if(data.getDesiredLeftFootsteps().size() <= 100)
      cdr.write_type_e(data.getDesiredLeftFootsteps());else
          throw new RuntimeException("desired_left_footsteps field exceeds the maximum length");

      if(data.getDesiredRightFootsteps().size() <= 100)
      cdr.write_type_e(data.getDesiredRightFootsteps());else
          throw new RuntimeException("desired_right_footsteps field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getCurrentLeftFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getCurrentRightFootPose(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_e(data.getFootsteps());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getLeftFootGoalPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getRightFootGoalPose(), cdr);	
      data.setExecutionState(cdr.read_type_9());
      	
      data.setTotalNumberOfFootsteps(cdr.read_type_3());
      	
      data.setNumberOfIncompleteFootsteps(cdr.read_type_3());
      	
      cdr.read_type_e(data.getDesiredLeftFootsteps());	
      cdr.read_type_e(data.getDesiredRightFootsteps());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getCurrentLeftFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getCurrentRightFootPose(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_e("footsteps", data.getFootsteps());
      ser.write_type_a("left_foot_goal_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftFootGoalPose());

      ser.write_type_a("right_foot_goal_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightFootGoalPose());

      ser.write_type_9("execution_state", data.getExecutionState());
      ser.write_type_3("total_number_of_footsteps", data.getTotalNumberOfFootsteps());
      ser.write_type_3("number_of_incomplete_footsteps", data.getNumberOfIncompleteFootsteps());
      ser.write_type_e("desired_left_footsteps", data.getDesiredLeftFootsteps());
      ser.write_type_e("desired_right_footsteps", data.getDesiredRightFootsteps());
      ser.write_type_a("current_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentLeftFootPose());

      ser.write_type_a("current_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentRightFootPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_e("footsteps", data.getFootsteps());
      ser.read_type_a("left_foot_goal_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftFootGoalPose());

      ser.read_type_a("right_foot_goal_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightFootGoalPose());

      data.setExecutionState(ser.read_type_9("execution_state"));
      data.setTotalNumberOfFootsteps(ser.read_type_3("total_number_of_footsteps"));
      data.setNumberOfIncompleteFootsteps(ser.read_type_3("number_of_incomplete_footsteps"));
      ser.read_type_e("desired_left_footsteps", data.getDesiredLeftFootsteps());
      ser.read_type_e("desired_right_footsteps", data.getDesiredRightFootsteps());
      ser.read_type_a("current_left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentLeftFootPose());

      ser.read_type_a("current_right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentRightFootPose());

   }

   public static void staticCopy(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage src, behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage createData()
   {
      return new behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage src, behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KickDoorApproachPlanStateMessagePubSubType newInstance()
   {
      return new KickDoorApproachPlanStateMessagePubSubType();
   }
}
