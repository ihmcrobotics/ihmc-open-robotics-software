package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanningToolboxOutputStatus" defined in "FootstepPlanningToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanningToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanningToolboxOutputStatus_.idl instead.
*
*/
public class FootstepPlanningToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlanningToolboxOutputStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f59d8310b8c6a4da79d23234e59e7a0a6fb88daba0a0ec96d60ba50a8cfbbe52";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getHeightMapMessage(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPath().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getBodyPath().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPathUnsmoothed().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBodyPathUnsmoothed().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getGoalPose(), current_alignment);

      current_alignment += toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType.getCdrSerializedSize(data.getPlannerTimings(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getExceptionMessage().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getStacktrace().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStacktrace().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getPlanId());

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootstepDataList(), cdr);
      cdr.write_type_9(data.getBodyPathPlanningResult());

      cdr.write_type_9(data.getFootstepPlanningResult());

      perception_msgs.msg.dds.HeightMapMessagePubSubType.write(data.getHeightMapMessage(), cdr);
      if(data.getBodyPath().size() <= 100)
      cdr.write_type_e(data.getBodyPath());else
          throw new RuntimeException("body_path field exceeds the maximum length");

      if(data.getBodyPathUnsmoothed().size() <= 100)
      cdr.write_type_e(data.getBodyPathUnsmoothed());else
          throw new RuntimeException("body_path_unsmoothed field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getGoalPose(), cdr);
      toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType.write(data.getPlannerTimings(), cdr);
      if(data.getExceptionMessage().length() <= 255)
      cdr.write_type_d(data.getExceptionMessage());else
          throw new RuntimeException("exception_message field exceeds the maximum length");

      if(data.getStacktrace().size() <= 20)
      cdr.write_type_e(data.getStacktrace());else
          throw new RuntimeException("stacktrace field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setPlanId(cdr.read_type_2());
      	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootstepDataList(), cdr);	
      data.setBodyPathPlanningResult(cdr.read_type_9());
      	
      data.setFootstepPlanningResult(cdr.read_type_9());
      	
      perception_msgs.msg.dds.HeightMapMessagePubSubType.read(data.getHeightMapMessage(), cdr);	
      cdr.read_type_e(data.getBodyPath());	
      cdr.read_type_e(data.getBodyPathUnsmoothed());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getGoalPose(), cdr);	
      toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType.read(data.getPlannerTimings(), cdr);	
      cdr.read_type_d(data.getExceptionMessage());	
      cdr.read_type_e(data.getStacktrace());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      ser.write_type_9("body_path_planning_result", data.getBodyPathPlanningResult());
      ser.write_type_9("footstep_planning_result", data.getFootstepPlanningResult());
      ser.write_type_a("height_map_message", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMapMessage());

      ser.write_type_e("body_path", data.getBodyPath());
      ser.write_type_e("body_path_unsmoothed", data.getBodyPathUnsmoothed());
      ser.write_type_a("goal_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalPose());

      ser.write_type_a("planner_timings", new toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType(), data.getPlannerTimings());

      ser.write_type_d("exception_message", data.getExceptionMessage());
      ser.write_type_e("stacktrace", data.getStacktrace());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      data.setBodyPathPlanningResult(ser.read_type_9("body_path_planning_result"));
      data.setFootstepPlanningResult(ser.read_type_9("footstep_planning_result"));
      ser.read_type_a("height_map_message", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMapMessage());

      ser.read_type_e("body_path", data.getBodyPath());
      ser.read_type_e("body_path_unsmoothed", data.getBodyPathUnsmoothed());
      ser.read_type_a("goal_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalPose());

      ser.read_type_a("planner_timings", new toolbox_msgs.msg.dds.FootstepPlanningTimingsMessagePubSubType(), data.getPlannerTimings());

      ser.read_type_d("exception_message", data.getExceptionMessage());
      ser.read_type_e("stacktrace", data.getStacktrace());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus src, toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus src, toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanningToolboxOutputStatusPubSubType newInstance()
   {
      return new FootstepPlanningToolboxOutputStatusPubSubType();
   }
}
