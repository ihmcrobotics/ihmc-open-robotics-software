package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedFootstepPlannerParametersPacket" defined in "QuadrupedFootstepPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedFootstepPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedFootstepPlannerParametersPacket_.idl instead.
*
*/
public class QuadrupedFootstepPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedFootstepPlannerParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getMaximumFrontStepReach());

      cdr.write_type_6(data.getMaximumFrontStepLength());

      cdr.write_type_6(data.getMinimumFrontStepLength());

      cdr.write_type_6(data.getMaximumHindStepReach());

      cdr.write_type_6(data.getMaximumHindStepLength());

      cdr.write_type_6(data.getMinimumHindStepLength());

      cdr.write_type_6(data.getMaximumStepWidth());

      cdr.write_type_6(data.getMinimumStepWidth());

      cdr.write_type_6(data.getMinimumStepYaw());

      cdr.write_type_6(data.getMaximumStepYaw());

      cdr.write_type_6(data.getMaximumStepChangeZ());

      cdr.write_type_6(data.getBodyGroundClearance());

      cdr.write_type_6(data.getDistanceHeuristicWeight());

      cdr.write_type_6(data.getYawWeight());

      cdr.write_type_6(data.getXGaitWeight());

      cdr.write_type_6(data.getCostPerStep());

      cdr.write_type_6(data.getStepUpWeight());

      cdr.write_type_6(data.getStepDownWeight());

      cdr.write_type_6(data.getHeuristicsWeight());

      cdr.write_type_6(data.getMinXClearanceFromFoot());

      cdr.write_type_6(data.getMinYClearanceFromFoot());

      cdr.write_type_6(data.getMaxWalkingSpeedMultiplier());

      cdr.write_type_7(data.getProjectInsideUsingConvexHullDuringExpansion());

      cdr.write_type_7(data.getProjectInsideUsingConvexHullDuringPostProcessing());

      cdr.write_type_6(data.getProjectionInsideDistanceForExpansion());

      cdr.write_type_6(data.getProjectionInsideDistanceForPostProcessing());

      cdr.write_type_6(data.getMaximumXyWiggleDistance());

      cdr.write_type_6(data.getMinimumSurfaceInclineRadians());

      cdr.write_type_6(data.getCliffHeightToAvoid());

      cdr.write_type_6(data.getMinimumDistanceFromCliffBottoms());

      cdr.write_type_6(data.getMinimumDistanceFromCliffTops());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setMaximumFrontStepReach(cdr.read_type_6());
      	
      data.setMaximumFrontStepLength(cdr.read_type_6());
      	
      data.setMinimumFrontStepLength(cdr.read_type_6());
      	
      data.setMaximumHindStepReach(cdr.read_type_6());
      	
      data.setMaximumHindStepLength(cdr.read_type_6());
      	
      data.setMinimumHindStepLength(cdr.read_type_6());
      	
      data.setMaximumStepWidth(cdr.read_type_6());
      	
      data.setMinimumStepWidth(cdr.read_type_6());
      	
      data.setMinimumStepYaw(cdr.read_type_6());
      	
      data.setMaximumStepYaw(cdr.read_type_6());
      	
      data.setMaximumStepChangeZ(cdr.read_type_6());
      	
      data.setBodyGroundClearance(cdr.read_type_6());
      	
      data.setDistanceHeuristicWeight(cdr.read_type_6());
      	
      data.setYawWeight(cdr.read_type_6());
      	
      data.setXGaitWeight(cdr.read_type_6());
      	
      data.setCostPerStep(cdr.read_type_6());
      	
      data.setStepUpWeight(cdr.read_type_6());
      	
      data.setStepDownWeight(cdr.read_type_6());
      	
      data.setHeuristicsWeight(cdr.read_type_6());
      	
      data.setMinXClearanceFromFoot(cdr.read_type_6());
      	
      data.setMinYClearanceFromFoot(cdr.read_type_6());
      	
      data.setMaxWalkingSpeedMultiplier(cdr.read_type_6());
      	
      data.setProjectInsideUsingConvexHullDuringExpansion(cdr.read_type_7());
      	
      data.setProjectInsideUsingConvexHullDuringPostProcessing(cdr.read_type_7());
      	
      data.setProjectionInsideDistanceForExpansion(cdr.read_type_6());
      	
      data.setProjectionInsideDistanceForPostProcessing(cdr.read_type_6());
      	
      data.setMaximumXyWiggleDistance(cdr.read_type_6());
      	
      data.setMinimumSurfaceInclineRadians(cdr.read_type_6());
      	
      data.setCliffHeightToAvoid(cdr.read_type_6());
      	
      data.setMinimumDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setMinimumDistanceFromCliffTops(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("maximum_front_step_reach", data.getMaximumFrontStepReach());
      ser.write_type_6("maximum_front_step_length", data.getMaximumFrontStepLength());
      ser.write_type_6("minimum_front_step_length", data.getMinimumFrontStepLength());
      ser.write_type_6("maximum_hind_step_reach", data.getMaximumHindStepReach());
      ser.write_type_6("maximum_hind_step_length", data.getMaximumHindStepLength());
      ser.write_type_6("minimum_hind_step_length", data.getMinimumHindStepLength());
      ser.write_type_6("maximum_step_width", data.getMaximumStepWidth());
      ser.write_type_6("minimum_step_width", data.getMinimumStepWidth());
      ser.write_type_6("minimum_step_yaw", data.getMinimumStepYaw());
      ser.write_type_6("maximum_step_yaw", data.getMaximumStepYaw());
      ser.write_type_6("maximum_step_change_z", data.getMaximumStepChangeZ());
      ser.write_type_6("body_ground_clearance", data.getBodyGroundClearance());
      ser.write_type_6("distance_heuristic_weight", data.getDistanceHeuristicWeight());
      ser.write_type_6("yaw_weight", data.getYawWeight());
      ser.write_type_6("x_gait_weight", data.getXGaitWeight());
      ser.write_type_6("cost_per_step", data.getCostPerStep());
      ser.write_type_6("step_up_weight", data.getStepUpWeight());
      ser.write_type_6("step_down_weight", data.getStepDownWeight());
      ser.write_type_6("heuristics_weight", data.getHeuristicsWeight());
      ser.write_type_6("min_x_clearance_from_foot", data.getMinXClearanceFromFoot());
      ser.write_type_6("min_y_clearance_from_foot", data.getMinYClearanceFromFoot());
      ser.write_type_6("max_walking_speed_multiplier", data.getMaxWalkingSpeedMultiplier());
      ser.write_type_7("project_inside_using_convex_hull_during_expansion", data.getProjectInsideUsingConvexHullDuringExpansion());
      ser.write_type_7("project_inside_using_convex_hull_during_post_processing", data.getProjectInsideUsingConvexHullDuringPostProcessing());
      ser.write_type_6("projection_inside_distance_for_expansion", data.getProjectionInsideDistanceForExpansion());
      ser.write_type_6("projection_inside_distance_for_post_processing", data.getProjectionInsideDistanceForPostProcessing());
      ser.write_type_6("maximum_xy_wiggle_distance", data.getMaximumXyWiggleDistance());
      ser.write_type_6("minimum_surface_incline_radians", data.getMinimumSurfaceInclineRadians());
      ser.write_type_6("cliff_height_to_avoid", data.getCliffHeightToAvoid());
      ser.write_type_6("minimum_distance_from_cliff_bottoms", data.getMinimumDistanceFromCliffBottoms());
      ser.write_type_6("minimum_distance_from_cliff_tops", data.getMinimumDistanceFromCliffTops());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setMaximumFrontStepReach(ser.read_type_6("maximum_front_step_reach"));
      data.setMaximumFrontStepLength(ser.read_type_6("maximum_front_step_length"));
      data.setMinimumFrontStepLength(ser.read_type_6("minimum_front_step_length"));
      data.setMaximumHindStepReach(ser.read_type_6("maximum_hind_step_reach"));
      data.setMaximumHindStepLength(ser.read_type_6("maximum_hind_step_length"));
      data.setMinimumHindStepLength(ser.read_type_6("minimum_hind_step_length"));
      data.setMaximumStepWidth(ser.read_type_6("maximum_step_width"));
      data.setMinimumStepWidth(ser.read_type_6("minimum_step_width"));
      data.setMinimumStepYaw(ser.read_type_6("minimum_step_yaw"));
      data.setMaximumStepYaw(ser.read_type_6("maximum_step_yaw"));
      data.setMaximumStepChangeZ(ser.read_type_6("maximum_step_change_z"));
      data.setBodyGroundClearance(ser.read_type_6("body_ground_clearance"));
      data.setDistanceHeuristicWeight(ser.read_type_6("distance_heuristic_weight"));
      data.setYawWeight(ser.read_type_6("yaw_weight"));
      data.setXGaitWeight(ser.read_type_6("x_gait_weight"));
      data.setCostPerStep(ser.read_type_6("cost_per_step"));
      data.setStepUpWeight(ser.read_type_6("step_up_weight"));
      data.setStepDownWeight(ser.read_type_6("step_down_weight"));
      data.setHeuristicsWeight(ser.read_type_6("heuristics_weight"));
      data.setMinXClearanceFromFoot(ser.read_type_6("min_x_clearance_from_foot"));
      data.setMinYClearanceFromFoot(ser.read_type_6("min_y_clearance_from_foot"));
      data.setMaxWalkingSpeedMultiplier(ser.read_type_6("max_walking_speed_multiplier"));
      data.setProjectInsideUsingConvexHullDuringExpansion(ser.read_type_7("project_inside_using_convex_hull_during_expansion"));
      data.setProjectInsideUsingConvexHullDuringPostProcessing(ser.read_type_7("project_inside_using_convex_hull_during_post_processing"));
      data.setProjectionInsideDistanceForExpansion(ser.read_type_6("projection_inside_distance_for_expansion"));
      data.setProjectionInsideDistanceForPostProcessing(ser.read_type_6("projection_inside_distance_for_post_processing"));
      data.setMaximumXyWiggleDistance(ser.read_type_6("maximum_xy_wiggle_distance"));
      data.setMinimumSurfaceInclineRadians(ser.read_type_6("minimum_surface_incline_radians"));
      data.setCliffHeightToAvoid(ser.read_type_6("cliff_height_to_avoid"));
      data.setMinimumDistanceFromCliffBottoms(ser.read_type_6("minimum_distance_from_cliff_bottoms"));
      data.setMinimumDistanceFromCliffTops(ser.read_type_6("minimum_distance_from_cliff_tops"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket src, controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket createData()
   {
      return new controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket src, controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedFootstepPlannerParametersPacketPubSubType newInstance()
   {
      return new QuadrupedFootstepPlannerParametersPacketPubSubType();
   }
}
