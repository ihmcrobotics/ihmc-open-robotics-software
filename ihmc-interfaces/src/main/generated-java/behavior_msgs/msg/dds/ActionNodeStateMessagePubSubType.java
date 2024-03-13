package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionNodeStateMessage" defined in "ActionNodeStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionNodeStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionNodeStateMessage_.idl instead.
*
*/
public class ActionNodeStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionNodeStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionNodeStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ae56056286fd79953a861b7481fa18d7e8e99c68e73f15c892963f054666224d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionNodeStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionNodeStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 500; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 7; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionNodeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionNodeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCommandedTrajectory().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getCommandedTrajectory().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCurrentPose(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCommandedJointTrajectories().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType.getCdrSerializedSize(data.getCommandedJointTrajectories().get(i0), current_alignment);}

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      cdr.write_type_4(data.getId());

      cdr.write_type_7(data.getIsNextForExecution());

      cdr.write_type_3(data.getConcurrencyRank());

      cdr.write_type_7(data.getCanExecute());

      cdr.write_type_7(data.getIsExecuting());

      cdr.write_type_7(data.getFailed());

      cdr.write_type_6(data.getNominalExecutionDuration());

      cdr.write_type_6(data.getElapsedExecutionTime());

      if(data.getCommandedTrajectory().size() <= 500)
      cdr.write_type_e(data.getCommandedTrajectory());else
          throw new RuntimeException("commanded_trajectory field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getCurrentPose(), cdr);
      if(data.getCommandedJointTrajectories().size() <= 7)
      cdr.write_type_e(data.getCommandedJointTrajectories());else
          throw new RuntimeException("commanded_joint_trajectories field exceeds the maximum length");

      for(int i0 = 0; i0 < data.getCurrentJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getCurrentJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getPositionDistanceToGoalTolerance());

      cdr.write_type_6(data.getOrientationDistanceToGoalTolerance());

   }

   public static void read(behavior_msgs.msg.dds.ActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      data.setId(cdr.read_type_4());
      	
      data.setIsNextForExecution(cdr.read_type_7());
      	
      data.setConcurrencyRank(cdr.read_type_3());
      	
      data.setCanExecute(cdr.read_type_7());
      	
      data.setIsExecuting(cdr.read_type_7());
      	
      data.setFailed(cdr.read_type_7());
      	
      data.setNominalExecutionDuration(cdr.read_type_6());
      	
      data.setElapsedExecutionTime(cdr.read_type_6());
      	
      cdr.read_type_e(data.getCommandedTrajectory());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getCurrentPose(), cdr);	
      cdr.read_type_e(data.getCommandedJointTrajectories());	
      for(int i0 = 0; i0 < data.getCurrentJointAngles().length; ++i0)
      {
        	data.getCurrentJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setPositionDistanceToGoalTolerance(cdr.read_type_6());
      	
      data.setOrientationDistanceToGoalTolerance(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionNodeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_4("id", data.getId());
      ser.write_type_7("is_next_for_execution", data.getIsNextForExecution());
      ser.write_type_3("concurrency_rank", data.getConcurrencyRank());
      ser.write_type_7("can_execute", data.getCanExecute());
      ser.write_type_7("is_executing", data.getIsExecuting());
      ser.write_type_7("failed", data.getFailed());
      ser.write_type_6("nominal_execution_duration", data.getNominalExecutionDuration());
      ser.write_type_6("elapsed_execution_time", data.getElapsedExecutionTime());
      ser.write_type_e("commanded_trajectory", data.getCommandedTrajectory());
      ser.write_type_a("current_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentPose());

      ser.write_type_e("commanded_joint_trajectories", data.getCommandedJointTrajectories());
      ser.write_type_f("current_joint_angles", data.getCurrentJointAngles());
      ser.write_type_6("position_distance_to_goal_tolerance", data.getPositionDistanceToGoalTolerance());
      ser.write_type_6("orientation_distance_to_goal_tolerance", data.getOrientationDistanceToGoalTolerance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionNodeStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      data.setId(ser.read_type_4("id"));
      data.setIsNextForExecution(ser.read_type_7("is_next_for_execution"));
      data.setConcurrencyRank(ser.read_type_3("concurrency_rank"));
      data.setCanExecute(ser.read_type_7("can_execute"));
      data.setIsExecuting(ser.read_type_7("is_executing"));
      data.setFailed(ser.read_type_7("failed"));
      data.setNominalExecutionDuration(ser.read_type_6("nominal_execution_duration"));
      data.setElapsedExecutionTime(ser.read_type_6("elapsed_execution_time"));
      ser.read_type_e("commanded_trajectory", data.getCommandedTrajectory());
      ser.read_type_a("current_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentPose());

      ser.read_type_e("commanded_joint_trajectories", data.getCommandedJointTrajectories());
      ser.read_type_f("current_joint_angles", data.getCurrentJointAngles());
      data.setPositionDistanceToGoalTolerance(ser.read_type_6("position_distance_to_goal_tolerance"));
      data.setOrientationDistanceToGoalTolerance(ser.read_type_6("orientation_distance_to_goal_tolerance"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionNodeStateMessage src, behavior_msgs.msg.dds.ActionNodeStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionNodeStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionNodeStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionNodeStateMessage src, behavior_msgs.msg.dds.ActionNodeStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionNodeStateMessagePubSubType newInstance()
   {
      return new ActionNodeStateMessagePubSubType();
   }
}
