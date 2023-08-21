package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BodyPathPlanMessage" defined in "BodyPathPlanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BodyPathPlanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BodyPathPlanMessage_.idl instead.
*
*/
public class BodyPathPlanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.BodyPathPlanMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::BodyPathPlanMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f946c1a87691219f82288488e08a1bbaaf945255a5af78e15d86c8ee9be35387";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.BodyPathPlanMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.BodyPathPlanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.BodyPathPlanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPath().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getBodyPath().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getCdrSerializedSize(data.getPathPlannerStartPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getCdrSerializedSize(data.getPathPlannerGoalPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFootstepPlanningResult());

      cdr.write_type_2(data.getPlanId());

      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsList(), cdr);
      if(data.getBodyPath().size() <= 100)
      cdr.write_type_e(data.getBodyPath());else
          throw new RuntimeException("body_path field exceeds the maximum length");

      geometry_msgs.msg.dds.Pose2DPubSubType.write(data.getPathPlannerStartPose(), cdr);
      geometry_msgs.msg.dds.Pose2DPubSubType.write(data.getPathPlannerGoalPose(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFootstepPlanningResult(cdr.read_type_9());
      	
      data.setPlanId(cdr.read_type_2());
      	
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsList(), cdr);	
      cdr.read_type_e(data.getBodyPath());	
      geometry_msgs.msg.dds.Pose2DPubSubType.read(data.getPathPlannerStartPose(), cdr);	
      geometry_msgs.msg.dds.Pose2DPubSubType.read(data.getPathPlannerGoalPose(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("footstep_planning_result", data.getFootstepPlanningResult());
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_a("planar_regions_list", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

      ser.write_type_e("body_path", data.getBodyPath());
      ser.write_type_a("path_planner_start_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerStartPose());

      ser.write_type_a("path_planner_goal_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerGoalPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.BodyPathPlanMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFootstepPlanningResult(ser.read_type_9("footstep_planning_result"));
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_a("planar_regions_list", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

      ser.read_type_e("body_path", data.getBodyPath());
      ser.read_type_a("path_planner_start_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerStartPose());

      ser.read_type_a("path_planner_goal_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerGoalPose());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.BodyPathPlanMessage src, toolbox_msgs.msg.dds.BodyPathPlanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.BodyPathPlanMessage createData()
   {
      return new toolbox_msgs.msg.dds.BodyPathPlanMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.BodyPathPlanMessage src, toolbox_msgs.msg.dds.BodyPathPlanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BodyPathPlanMessagePubSubType newInstance()
   {
      return new BodyPathPlanMessagePubSubType();
   }
}
