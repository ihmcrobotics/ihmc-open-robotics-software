package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PawStepPlannerParametersPacket" defined in "PawStepPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PawStepPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PawStepPlannerParametersPacket_.idl instead.
*
*/
public class PawStepPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.PawStepPlannerParametersPacket>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::PawStepPlannerParametersPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9824842df5e9f3d2b60fabe94d005f892a4ddb451552d58fd07872476e2ec8f7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, int current_alignment)
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getMaximumFrontStepReach());

      cdr.write_type_6(data.getMaximumFrontStepLength());

      cdr.write_type_6(data.getMinimumFrontStepLength());

      cdr.write_type_6(data.getMaximumHindStepReach());

      cdr.write_type_6(data.getMaximumHindStepLength());

      cdr.write_type_6(data.getMinimumHindStepLength());

      cdr.write_type_6(data.getMaximumFrontStepLengthWhenSteppingUp());

      cdr.write_type_6(data.getMinimumFrontStepLengthWhenSteppingUp());

      cdr.write_type_6(data.getMaximumHindStepLengthWhenSteppingUp());

      cdr.write_type_6(data.getMinimumHindStepLengthWhenSteppingUp());

      cdr.write_type_6(data.getStepZForSteppingUp());

      cdr.write_type_6(data.getMaximumFrontStepLengthWhenSteppingDown());

      cdr.write_type_6(data.getMinimumFrontStepLengthWhenSteppingDown());

      cdr.write_type_6(data.getMaximumHindStepLengthWhenSteppingDown());

      cdr.write_type_6(data.getMinimumHindStepLengthWhenSteppingDown());

      cdr.write_type_6(data.getStepZForSteppingDown());

      cdr.write_type_6(data.getMaximumStepOutward());

      cdr.write_type_6(data.getMaximumStepInward());

      cdr.write_type_6(data.getMaximumStepYawOutward());

      cdr.write_type_6(data.getMaximumStepYawInward());

      cdr.write_type_6(data.getMaximumStepChangeZ());

      cdr.write_type_6(data.getBodyGroundClearance());

      cdr.write_type_6(data.getDistanceWeight());

      cdr.write_type_6(data.getYawWeight());

      cdr.write_type_6(data.getXGaitWeight());

      cdr.write_type_6(data.getDesiredVelocityWeight());

      cdr.write_type_6(data.getCostPerStep());

      cdr.write_type_6(data.getStepUpWeight());

      cdr.write_type_6(data.getStepDownWeight());

      cdr.write_type_6(data.getHeuristicsWeight());

      cdr.write_type_6(data.getMinXClearanceFromPaw());

      cdr.write_type_6(data.getMinYClearanceFromPaw());

      cdr.write_type_6(data.getMaxWalkingSpeedMultiplier());

      cdr.write_type_7(data.getProjectInsideUsingConvexHull());

      cdr.write_type_6(data.getProjectionInsideDistance());

      cdr.write_type_6(data.getMinimumProjectionInsideDistance());

      cdr.write_type_6(data.getMaximumXyWiggleDistance());

      cdr.write_type_6(data.getMinimumSurfaceInclineRadians());

      cdr.write_type_6(data.getCliffHeightToAvoid());

      cdr.write_type_6(data.getMinimumFrontEndForwardDistanceFromCliffBottoms());

      cdr.write_type_6(data.getMinimumFrontEndBackwardDistanceFromCliffBottoms());

      cdr.write_type_6(data.getMinimumHindEndForwardDistanceFromCliffBottoms());

      cdr.write_type_6(data.getMinimumHindEndBackwardDistanceFromCliffBottoms());

      cdr.write_type_6(data.getMinimumLateralDistanceFromCliffBottoms());

      cdr.write_type_6(data.getFinalTurnProximity());

      cdr.write_type_6(data.getFinalSlowDownProximity());

      cdr.write_type_6(data.getMaximumDeviationFromXGaitDuringExpansion());

      cdr.write_type_7(data.getReturnBestEffortPlan());

      cdr.write_type_4(data.getMinimumStepsForBestEffortPlan());

      cdr.write_type_7(data.getPerformGraphRepairingStep());

      cdr.write_type_6(data.getRepairingHeuristicWeightScaling());

      cdr.write_type_6(data.getMinimumHeuristicWeightReduction());

   }

   public static void read(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setMaximumFrontStepReach(cdr.read_type_6());
      	
      data.setMaximumFrontStepLength(cdr.read_type_6());
      	
      data.setMinimumFrontStepLength(cdr.read_type_6());
      	
      data.setMaximumHindStepReach(cdr.read_type_6());
      	
      data.setMaximumHindStepLength(cdr.read_type_6());
      	
      data.setMinimumHindStepLength(cdr.read_type_6());
      	
      data.setMaximumFrontStepLengthWhenSteppingUp(cdr.read_type_6());
      	
      data.setMinimumFrontStepLengthWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumHindStepLengthWhenSteppingUp(cdr.read_type_6());
      	
      data.setMinimumHindStepLengthWhenSteppingUp(cdr.read_type_6());
      	
      data.setStepZForSteppingUp(cdr.read_type_6());
      	
      data.setMaximumFrontStepLengthWhenSteppingDown(cdr.read_type_6());
      	
      data.setMinimumFrontStepLengthWhenSteppingDown(cdr.read_type_6());
      	
      data.setMaximumHindStepLengthWhenSteppingDown(cdr.read_type_6());
      	
      data.setMinimumHindStepLengthWhenSteppingDown(cdr.read_type_6());
      	
      data.setStepZForSteppingDown(cdr.read_type_6());
      	
      data.setMaximumStepOutward(cdr.read_type_6());
      	
      data.setMaximumStepInward(cdr.read_type_6());
      	
      data.setMaximumStepYawOutward(cdr.read_type_6());
      	
      data.setMaximumStepYawInward(cdr.read_type_6());
      	
      data.setMaximumStepChangeZ(cdr.read_type_6());
      	
      data.setBodyGroundClearance(cdr.read_type_6());
      	
      data.setDistanceWeight(cdr.read_type_6());
      	
      data.setYawWeight(cdr.read_type_6());
      	
      data.setXGaitWeight(cdr.read_type_6());
      	
      data.setDesiredVelocityWeight(cdr.read_type_6());
      	
      data.setCostPerStep(cdr.read_type_6());
      	
      data.setStepUpWeight(cdr.read_type_6());
      	
      data.setStepDownWeight(cdr.read_type_6());
      	
      data.setHeuristicsWeight(cdr.read_type_6());
      	
      data.setMinXClearanceFromPaw(cdr.read_type_6());
      	
      data.setMinYClearanceFromPaw(cdr.read_type_6());
      	
      data.setMaxWalkingSpeedMultiplier(cdr.read_type_6());
      	
      data.setProjectInsideUsingConvexHull(cdr.read_type_7());
      	
      data.setProjectionInsideDistance(cdr.read_type_6());
      	
      data.setMinimumProjectionInsideDistance(cdr.read_type_6());
      	
      data.setMaximumXyWiggleDistance(cdr.read_type_6());
      	
      data.setMinimumSurfaceInclineRadians(cdr.read_type_6());
      	
      data.setCliffHeightToAvoid(cdr.read_type_6());
      	
      data.setMinimumFrontEndForwardDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setMinimumFrontEndBackwardDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setMinimumHindEndForwardDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setMinimumHindEndBackwardDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setMinimumLateralDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setFinalTurnProximity(cdr.read_type_6());
      	
      data.setFinalSlowDownProximity(cdr.read_type_6());
      	
      data.setMaximumDeviationFromXGaitDuringExpansion(cdr.read_type_6());
      	
      data.setReturnBestEffortPlan(cdr.read_type_7());
      	
      data.setMinimumStepsForBestEffortPlan(cdr.read_type_4());
      	
      data.setPerformGraphRepairingStep(cdr.read_type_7());
      	
      data.setRepairingHeuristicWeightScaling(cdr.read_type_6());
      	
      data.setMinimumHeuristicWeightReduction(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("maximum_front_step_reach", data.getMaximumFrontStepReach());
      ser.write_type_6("maximum_front_step_length", data.getMaximumFrontStepLength());
      ser.write_type_6("minimum_front_step_length", data.getMinimumFrontStepLength());
      ser.write_type_6("maximum_hind_step_reach", data.getMaximumHindStepReach());
      ser.write_type_6("maximum_hind_step_length", data.getMaximumHindStepLength());
      ser.write_type_6("minimum_hind_step_length", data.getMinimumHindStepLength());
      ser.write_type_6("maximum_front_step_length_when_stepping_up", data.getMaximumFrontStepLengthWhenSteppingUp());
      ser.write_type_6("minimum_front_step_length_when_stepping_up", data.getMinimumFrontStepLengthWhenSteppingUp());
      ser.write_type_6("maximum_hind_step_length_when_stepping_up", data.getMaximumHindStepLengthWhenSteppingUp());
      ser.write_type_6("minimum_hind_step_length_when_stepping_up", data.getMinimumHindStepLengthWhenSteppingUp());
      ser.write_type_6("step_z_for_stepping_up", data.getStepZForSteppingUp());
      ser.write_type_6("maximum_front_step_length_when_stepping_down", data.getMaximumFrontStepLengthWhenSteppingDown());
      ser.write_type_6("minimum_front_step_length_when_stepping_down", data.getMinimumFrontStepLengthWhenSteppingDown());
      ser.write_type_6("maximum_hind_step_length_when_stepping_down", data.getMaximumHindStepLengthWhenSteppingDown());
      ser.write_type_6("minimum_hind_step_length_when_stepping_down", data.getMinimumHindStepLengthWhenSteppingDown());
      ser.write_type_6("step_z_for_stepping_down", data.getStepZForSteppingDown());
      ser.write_type_6("maximum_step_outward", data.getMaximumStepOutward());
      ser.write_type_6("maximum_step_inward", data.getMaximumStepInward());
      ser.write_type_6("maximum_step_yaw_outward", data.getMaximumStepYawOutward());
      ser.write_type_6("maximum_step_yaw_inward", data.getMaximumStepYawInward());
      ser.write_type_6("maximum_step_change_z", data.getMaximumStepChangeZ());
      ser.write_type_6("body_ground_clearance", data.getBodyGroundClearance());
      ser.write_type_6("distance_weight", data.getDistanceWeight());
      ser.write_type_6("yaw_weight", data.getYawWeight());
      ser.write_type_6("x_gait_weight", data.getXGaitWeight());
      ser.write_type_6("desired_velocity_weight", data.getDesiredVelocityWeight());
      ser.write_type_6("cost_per_step", data.getCostPerStep());
      ser.write_type_6("step_up_weight", data.getStepUpWeight());
      ser.write_type_6("step_down_weight", data.getStepDownWeight());
      ser.write_type_6("heuristics_weight", data.getHeuristicsWeight());
      ser.write_type_6("min_x_clearance_from_paw", data.getMinXClearanceFromPaw());
      ser.write_type_6("min_y_clearance_from_paw", data.getMinYClearanceFromPaw());
      ser.write_type_6("max_walking_speed_multiplier", data.getMaxWalkingSpeedMultiplier());
      ser.write_type_7("project_inside_using_convex_hull", data.getProjectInsideUsingConvexHull());
      ser.write_type_6("projection_inside_distance", data.getProjectionInsideDistance());
      ser.write_type_6("minimum_projection_inside_distance", data.getMinimumProjectionInsideDistance());
      ser.write_type_6("maximum_xy_wiggle_distance", data.getMaximumXyWiggleDistance());
      ser.write_type_6("minimum_surface_incline_radians", data.getMinimumSurfaceInclineRadians());
      ser.write_type_6("cliff_height_to_avoid", data.getCliffHeightToAvoid());
      ser.write_type_6("minimum_front_end_forward_distance_from_cliff_bottoms", data.getMinimumFrontEndForwardDistanceFromCliffBottoms());
      ser.write_type_6("minimum_front_end_backward_distance_from_cliff_bottoms", data.getMinimumFrontEndBackwardDistanceFromCliffBottoms());
      ser.write_type_6("minimum_hind_end_forward_distance_from_cliff_bottoms", data.getMinimumHindEndForwardDistanceFromCliffBottoms());
      ser.write_type_6("minimum_hind_end_backward_distance_from_cliff_bottoms", data.getMinimumHindEndBackwardDistanceFromCliffBottoms());
      ser.write_type_6("minimum_lateral_distance_from_cliff_bottoms", data.getMinimumLateralDistanceFromCliffBottoms());
      ser.write_type_6("final_turn_proximity", data.getFinalTurnProximity());
      ser.write_type_6("final_slow_down_proximity", data.getFinalSlowDownProximity());
      ser.write_type_6("maximum_deviation_from_x_gait_during_expansion", data.getMaximumDeviationFromXGaitDuringExpansion());
      ser.write_type_7("return_best_effort_plan", data.getReturnBestEffortPlan());
      ser.write_type_4("minimum_steps_for_best_effort_plan", data.getMinimumStepsForBestEffortPlan());
      ser.write_type_7("perform_graph_repairing_step", data.getPerformGraphRepairingStep());
      ser.write_type_6("repairing_heuristic_weight_scaling", data.getRepairingHeuristicWeightScaling());
      ser.write_type_6("minimum_heuristic_weight_reduction", data.getMinimumHeuristicWeightReduction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setMaximumFrontStepReach(ser.read_type_6("maximum_front_step_reach"));
      data.setMaximumFrontStepLength(ser.read_type_6("maximum_front_step_length"));
      data.setMinimumFrontStepLength(ser.read_type_6("minimum_front_step_length"));
      data.setMaximumHindStepReach(ser.read_type_6("maximum_hind_step_reach"));
      data.setMaximumHindStepLength(ser.read_type_6("maximum_hind_step_length"));
      data.setMinimumHindStepLength(ser.read_type_6("minimum_hind_step_length"));
      data.setMaximumFrontStepLengthWhenSteppingUp(ser.read_type_6("maximum_front_step_length_when_stepping_up"));
      data.setMinimumFrontStepLengthWhenSteppingUp(ser.read_type_6("minimum_front_step_length_when_stepping_up"));
      data.setMaximumHindStepLengthWhenSteppingUp(ser.read_type_6("maximum_hind_step_length_when_stepping_up"));
      data.setMinimumHindStepLengthWhenSteppingUp(ser.read_type_6("minimum_hind_step_length_when_stepping_up"));
      data.setStepZForSteppingUp(ser.read_type_6("step_z_for_stepping_up"));
      data.setMaximumFrontStepLengthWhenSteppingDown(ser.read_type_6("maximum_front_step_length_when_stepping_down"));
      data.setMinimumFrontStepLengthWhenSteppingDown(ser.read_type_6("minimum_front_step_length_when_stepping_down"));
      data.setMaximumHindStepLengthWhenSteppingDown(ser.read_type_6("maximum_hind_step_length_when_stepping_down"));
      data.setMinimumHindStepLengthWhenSteppingDown(ser.read_type_6("minimum_hind_step_length_when_stepping_down"));
      data.setStepZForSteppingDown(ser.read_type_6("step_z_for_stepping_down"));
      data.setMaximumStepOutward(ser.read_type_6("maximum_step_outward"));
      data.setMaximumStepInward(ser.read_type_6("maximum_step_inward"));
      data.setMaximumStepYawOutward(ser.read_type_6("maximum_step_yaw_outward"));
      data.setMaximumStepYawInward(ser.read_type_6("maximum_step_yaw_inward"));
      data.setMaximumStepChangeZ(ser.read_type_6("maximum_step_change_z"));
      data.setBodyGroundClearance(ser.read_type_6("body_ground_clearance"));
      data.setDistanceWeight(ser.read_type_6("distance_weight"));
      data.setYawWeight(ser.read_type_6("yaw_weight"));
      data.setXGaitWeight(ser.read_type_6("x_gait_weight"));
      data.setDesiredVelocityWeight(ser.read_type_6("desired_velocity_weight"));
      data.setCostPerStep(ser.read_type_6("cost_per_step"));
      data.setStepUpWeight(ser.read_type_6("step_up_weight"));
      data.setStepDownWeight(ser.read_type_6("step_down_weight"));
      data.setHeuristicsWeight(ser.read_type_6("heuristics_weight"));
      data.setMinXClearanceFromPaw(ser.read_type_6("min_x_clearance_from_paw"));
      data.setMinYClearanceFromPaw(ser.read_type_6("min_y_clearance_from_paw"));
      data.setMaxWalkingSpeedMultiplier(ser.read_type_6("max_walking_speed_multiplier"));
      data.setProjectInsideUsingConvexHull(ser.read_type_7("project_inside_using_convex_hull"));
      data.setProjectionInsideDistance(ser.read_type_6("projection_inside_distance"));
      data.setMinimumProjectionInsideDistance(ser.read_type_6("minimum_projection_inside_distance"));
      data.setMaximumXyWiggleDistance(ser.read_type_6("maximum_xy_wiggle_distance"));
      data.setMinimumSurfaceInclineRadians(ser.read_type_6("minimum_surface_incline_radians"));
      data.setCliffHeightToAvoid(ser.read_type_6("cliff_height_to_avoid"));
      data.setMinimumFrontEndForwardDistanceFromCliffBottoms(ser.read_type_6("minimum_front_end_forward_distance_from_cliff_bottoms"));
      data.setMinimumFrontEndBackwardDistanceFromCliffBottoms(ser.read_type_6("minimum_front_end_backward_distance_from_cliff_bottoms"));
      data.setMinimumHindEndForwardDistanceFromCliffBottoms(ser.read_type_6("minimum_hind_end_forward_distance_from_cliff_bottoms"));
      data.setMinimumHindEndBackwardDistanceFromCliffBottoms(ser.read_type_6("minimum_hind_end_backward_distance_from_cliff_bottoms"));
      data.setMinimumLateralDistanceFromCliffBottoms(ser.read_type_6("minimum_lateral_distance_from_cliff_bottoms"));
      data.setFinalTurnProximity(ser.read_type_6("final_turn_proximity"));
      data.setFinalSlowDownProximity(ser.read_type_6("final_slow_down_proximity"));
      data.setMaximumDeviationFromXGaitDuringExpansion(ser.read_type_6("maximum_deviation_from_x_gait_during_expansion"));
      data.setReturnBestEffortPlan(ser.read_type_7("return_best_effort_plan"));
      data.setMinimumStepsForBestEffortPlan(ser.read_type_4("minimum_steps_for_best_effort_plan"));
      data.setPerformGraphRepairingStep(ser.read_type_7("perform_graph_repairing_step"));
      data.setRepairingHeuristicWeightScaling(ser.read_type_6("repairing_heuristic_weight_scaling"));
      data.setMinimumHeuristicWeightReduction(ser.read_type_6("minimum_heuristic_weight_reduction"));
   }

   public static void staticCopy(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket src, quadruped_msgs.msg.dds.PawStepPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.PawStepPlannerParametersPacket createData()
   {
      return new quadruped_msgs.msg.dds.PawStepPlannerParametersPacket();
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
   
   public void serialize(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.PawStepPlannerParametersPacket src, quadruped_msgs.msg.dds.PawStepPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PawStepPlannerParametersPacketPubSubType newInstance()
   {
      return new PawStepPlannerParametersPacketPubSubType();
   }
}
