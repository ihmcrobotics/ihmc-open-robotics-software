package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BodyPathPlanMessage" defined in "BodyPathPlanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BodyPathPlanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BodyPathPlanMessage_.idl instead.
*
*/
public class BodyPathPlanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BodyPathPlanMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BodyPathPlanMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BodyPathPlanMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BodyPathPlanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BodyPathPlanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPath().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getBodyPath().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getCdrSerializedSize(data.getPathPlannerStartPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Pose2DPubSubType.getCdrSerializedSize(data.getPathPlannerGoalPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFootstepPlanningResult());

      cdr.write_type_2(data.getPlanId());

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsList(), cdr);
      if(data.getBodyPath().size() <= 100)
      cdr.write_type_e(data.getBodyPath());else
          throw new RuntimeException("body_path field exceeds the maximum length");

      geometry_msgs.msg.dds.Pose2DPubSubType.write(data.getPathPlannerStartPose(), cdr);
      geometry_msgs.msg.dds.Pose2DPubSubType.write(data.getPathPlannerGoalPose(), cdr);
   }

   public static void read(controller_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFootstepPlanningResult(cdr.read_type_9());
      	
      data.setPlanId(cdr.read_type_2());
      	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsList(), cdr);	
      cdr.read_type_e(data.getBodyPath());	
      geometry_msgs.msg.dds.Pose2DPubSubType.read(data.getPathPlannerStartPose(), cdr);	
      geometry_msgs.msg.dds.Pose2DPubSubType.read(data.getPathPlannerGoalPose(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("footstep_planning_result", data.getFootstepPlanningResult());
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

      ser.write_type_e("body_path", data.getBodyPath());
      ser.write_type_a("path_planner_start_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerStartPose());

      ser.write_type_a("path_planner_goal_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerGoalPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BodyPathPlanMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFootstepPlanningResult(ser.read_type_9("footstep_planning_result"));
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

      ser.read_type_e("body_path", data.getBodyPath());
      ser.read_type_a("path_planner_start_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerStartPose());

      ser.read_type_a("path_planner_goal_pose", new geometry_msgs.msg.dds.Pose2DPubSubType(), data.getPathPlannerGoalPose());

   }

   public static void staticCopy(controller_msgs.msg.dds.BodyPathPlanMessage src, controller_msgs.msg.dds.BodyPathPlanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BodyPathPlanMessage createData()
   {
      return new controller_msgs.msg.dds.BodyPathPlanMessage();
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
   
   public void serialize(controller_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BodyPathPlanMessage src, controller_msgs.msg.dds.BodyPathPlanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BodyPathPlanMessagePubSubType newInstance()
   {
      return new BodyPathPlanMessagePubSubType();
   }
}
