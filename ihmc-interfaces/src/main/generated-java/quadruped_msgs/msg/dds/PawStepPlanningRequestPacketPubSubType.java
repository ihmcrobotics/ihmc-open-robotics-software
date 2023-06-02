package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PawStepPlanningRequestPacket" defined in "PawStepPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PawStepPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PawStepPlanningRequestPacket_.idl instead.
*
*/
public class PawStepPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.PawStepPlanningRequestPacket>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::PawStepPlanningRequestPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d5a261a0f8296480bd57b7f6cd9c4006e7b5180e6a19b6110d2f6a912c02a77c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBodyPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getBodyOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getFrontLeftPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getFrontRightPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getHindLeftPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getHindRightPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getGoalOrientationInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsListMessage(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getInitialStepRobotQuadrant());

      cdr.write_type_9(data.getStartTargetType());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getBodyPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getBodyOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getFrontLeftPositionInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getFrontRightPositionInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getHindLeftPositionInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getHindRightPositionInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getGoalOrientationInWorld(), cdr);
      cdr.write_type_2(data.getPlannerRequestId());

      cdr.write_type_9(data.getRequestedPawPlannerType());

      cdr.write_type_6(data.getTimeout());

      cdr.write_type_6(data.getBestEffortTimeout());

      cdr.write_type_6(data.getHorizonLength());

      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsListMessage(), cdr);
      cdr.write_type_7(data.getAssumeFlatGround());

   }

   public static void read(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setInitialStepRobotQuadrant(cdr.read_type_9());
      	
      data.setStartTargetType(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getBodyPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getBodyOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getFrontLeftPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getFrontRightPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getHindLeftPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getHindRightPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getGoalOrientationInWorld(), cdr);	
      data.setPlannerRequestId(cdr.read_type_2());
      	
      data.setRequestedPawPlannerType(cdr.read_type_9());
      	
      data.setTimeout(cdr.read_type_6());
      	
      data.setBestEffortTimeout(cdr.read_type_6());
      	
      data.setHorizonLength(cdr.read_type_6());
      	
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsListMessage(), cdr);	
      data.setAssumeFlatGround(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("initial_step_robot_quadrant", data.getInitialStepRobotQuadrant());
      ser.write_type_9("start_target_type", data.getStartTargetType());
      ser.write_type_a("body_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getBodyPositionInWorld());

      ser.write_type_a("body_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getBodyOrientationInWorld());

      ser.write_type_a("front_left_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getFrontLeftPositionInWorld());

      ser.write_type_a("front_right_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getFrontRightPositionInWorld());

      ser.write_type_a("hind_left_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getHindLeftPositionInWorld());

      ser.write_type_a("hind_right_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getHindRightPositionInWorld());

      ser.write_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.write_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      ser.write_type_2("planner_request_id", data.getPlannerRequestId());
      ser.write_type_9("requested_paw_planner_type", data.getRequestedPawPlannerType());
      ser.write_type_6("timeout", data.getTimeout());
      ser.write_type_6("best_effort_timeout", data.getBestEffortTimeout());
      ser.write_type_6("horizon_length", data.getHorizonLength());
      ser.write_type_a("planar_regions_list_message", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      ser.write_type_7("assume_flat_ground", data.getAssumeFlatGround());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setInitialStepRobotQuadrant(ser.read_type_9("initial_step_robot_quadrant"));
      data.setStartTargetType(ser.read_type_9("start_target_type"));
      ser.read_type_a("body_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getBodyPositionInWorld());

      ser.read_type_a("body_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getBodyOrientationInWorld());

      ser.read_type_a("front_left_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getFrontLeftPositionInWorld());

      ser.read_type_a("front_right_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getFrontRightPositionInWorld());

      ser.read_type_a("hind_left_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getHindLeftPositionInWorld());

      ser.read_type_a("hind_right_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getHindRightPositionInWorld());

      ser.read_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.read_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      data.setPlannerRequestId(ser.read_type_2("planner_request_id"));
      data.setRequestedPawPlannerType(ser.read_type_9("requested_paw_planner_type"));
      data.setTimeout(ser.read_type_6("timeout"));
      data.setBestEffortTimeout(ser.read_type_6("best_effort_timeout"));
      data.setHorizonLength(ser.read_type_6("horizon_length"));
      ser.read_type_a("planar_regions_list_message", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsListMessage());

      data.setAssumeFlatGround(ser.read_type_7("assume_flat_ground"));
   }

   public static void staticCopy(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket src, quadruped_msgs.msg.dds.PawStepPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.PawStepPlanningRequestPacket createData()
   {
      return new quadruped_msgs.msg.dds.PawStepPlanningRequestPacket();
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
   
   public void serialize(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.PawStepPlanningRequestPacket src, quadruped_msgs.msg.dds.PawStepPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PawStepPlanningRequestPacketPubSubType newInstance()
   {
      return new PawStepPlanningRequestPacketPubSubType();
   }
}
