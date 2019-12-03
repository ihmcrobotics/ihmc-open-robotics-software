package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ValkyrieFootstepPlanningRequestPacket" defined in "ValkyrieFootstepPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ValkyrieFootstepPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ValkyrieFootstepPlanningRequestPacket_.idl instead.
*
*/
public class ValkyrieFootstepPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ValkyrieFootstepPlanningRequestPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getLeftFootPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getRightFootPose(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getGoalPoses().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getGoalPoses().get(i0), current_alignment);}

      current_alignment += controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType.getCdrSerializedSize(data.getParameters(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsListMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getPlannerRequestId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getLeftFootPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getRightFootPose(), cdr);
      if(data.getGoalPoses().size() <= 100)
      cdr.write_type_e(data.getGoalPoses());else
          throw new RuntimeException("goal_poses field exceeds the maximum length");

      controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType.write(data.getParameters(), cdr);
      cdr.write_type_6(data.getGoalDistanceProximity());

      cdr.write_type_6(data.getGoalYawProximity());

      cdr.write_type_6(data.getTimeout());

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsListMessage(), cdr);
      cdr.write_type_7(data.getAssumeFlatGround());

      cdr.write_type_7(data.getComputeBodyPath());

   }

   public static void read(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setPlannerRequestId(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getLeftFootPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getRightFootPose(), cdr);	
      cdr.read_type_e(data.getGoalPoses());	
      controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType.read(data.getParameters(), cdr);	
      data.setGoalDistanceProximity(cdr.read_type_6());
      	
      data.setGoalYawProximity(cdr.read_type_6());
      	
      data.setTimeout(cdr.read_type_6());
      	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsListMessage(), cdr);	
      data.setAssumeFlatGround(cdr.read_type_7());
      	
      data.setComputeBodyPath(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("planner_request_id", data.getPlannerRequestId());
      ser.write_type_a("left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftFootPose());

      ser.write_type_a("right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightFootPose());

      ser.write_type_e("goal_poses", data.getGoalPoses());
      ser.write_type_a("parameters", new controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType(), data.getParameters());

      ser.write_type_6("goal_distance_proximity", data.getGoalDistanceProximity());
      ser.write_type_6("goal_yaw_proximity", data.getGoalYawProximity());
      ser.write_type_6("timeout", data.getTimeout());
      ser.write_type_a("planar_regions_list_message", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      ser.write_type_7("assume_flat_ground", data.getAssumeFlatGround());
      ser.write_type_7("compute_body_path", data.getComputeBodyPath());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data)
   {
      data.setPlannerRequestId(ser.read_type_2("planner_request_id"));
      ser.read_type_a("left_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getLeftFootPose());

      ser.read_type_a("right_foot_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRightFootPose());

      ser.read_type_e("goal_poses", data.getGoalPoses());
      ser.read_type_a("parameters", new controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType(), data.getParameters());

      data.setGoalDistanceProximity(ser.read_type_6("goal_distance_proximity"));
      data.setGoalYawProximity(ser.read_type_6("goal_yaw_proximity"));
      data.setTimeout(ser.read_type_6("timeout"));
      ser.read_type_a("planar_regions_list_message", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      data.setAssumeFlatGround(ser.read_type_7("assume_flat_ground"));
      data.setComputeBodyPath(ser.read_type_7("compute_body_path"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket src, controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket createData()
   {
      return new controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket();
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
   
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket src, controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ValkyrieFootstepPlanningRequestPacketPubSubType newInstance()
   {
      return new ValkyrieFootstepPlanningRequestPacketPubSubType();
   }
}
