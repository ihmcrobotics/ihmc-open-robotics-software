package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AStarBodyPathPlannerParametersPacket" defined in "AStarBodyPathPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AStarBodyPathPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AStarBodyPathPlannerParametersPacket_.idl instead.
*
*/
public class AStarBodyPathPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::AStarBodyPathPlannerParametersPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "78135a961eb2e81e643a17583e342085f18feb62d009cfbef14c337d46fdfc48";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_7(data.getCheckForCollisions());

      cdr.write_type_7(data.getPerformSmoothing());

      cdr.write_type_6(data.getSnapRadius());

      cdr.write_type_6(data.getMinSnapHeightThreshold());

      cdr.write_type_6(data.getInclineCostWeight());

      cdr.write_type_6(data.getInclineCostDeadband());

      cdr.write_type_6(data.getMaxIncline());

      cdr.write_type_6(data.getCollisionBoxSizeY());

      cdr.write_type_6(data.getCollisionBoxSizeX());

      cdr.write_type_6(data.getCollisionBoxGroundClearance());

      cdr.write_type_6(data.getSmootherCollisionWeight());

      cdr.write_type_6(data.getSmootherSmoothnessWeight());

      cdr.write_type_6(data.getSmootherTurnPointSmoothnessDiscount());

      cdr.write_type_6(data.getSmootherMinCurvatureToPenalize());

      cdr.write_type_6(data.getSmootherEqualSpacingWeight());

      cdr.write_type_6(data.getSmootherDisplacementWeight());

      cdr.write_type_6(data.getSmootherHillClimbGain());

      cdr.write_type_6(data.getSmootherGradientThresholdToTerminate());

   }

   public static void read(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setCheckForCollisions(cdr.read_type_7());
      	
      data.setPerformSmoothing(cdr.read_type_7());
      	
      data.setSnapRadius(cdr.read_type_6());
      	
      data.setMinSnapHeightThreshold(cdr.read_type_6());
      	
      data.setInclineCostWeight(cdr.read_type_6());
      	
      data.setInclineCostDeadband(cdr.read_type_6());
      	
      data.setMaxIncline(cdr.read_type_6());
      	
      data.setCollisionBoxSizeY(cdr.read_type_6());
      	
      data.setCollisionBoxSizeX(cdr.read_type_6());
      	
      data.setCollisionBoxGroundClearance(cdr.read_type_6());
      	
      data.setSmootherCollisionWeight(cdr.read_type_6());
      	
      data.setSmootherSmoothnessWeight(cdr.read_type_6());
      	
      data.setSmootherTurnPointSmoothnessDiscount(cdr.read_type_6());
      	
      data.setSmootherMinCurvatureToPenalize(cdr.read_type_6());
      	
      data.setSmootherEqualSpacingWeight(cdr.read_type_6());
      	
      data.setSmootherDisplacementWeight(cdr.read_type_6());
      	
      data.setSmootherHillClimbGain(cdr.read_type_6());
      	
      data.setSmootherGradientThresholdToTerminate(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_7("check_for_collisions", data.getCheckForCollisions());
      ser.write_type_7("perform_smoothing", data.getPerformSmoothing());
      ser.write_type_6("snap_radius", data.getSnapRadius());
      ser.write_type_6("min_snap_height_threshold", data.getMinSnapHeightThreshold());
      ser.write_type_6("incline_cost_weight", data.getInclineCostWeight());
      ser.write_type_6("incline_cost_deadband", data.getInclineCostDeadband());
      ser.write_type_6("max_incline", data.getMaxIncline());
      ser.write_type_6("collision_box_size_y", data.getCollisionBoxSizeY());
      ser.write_type_6("collision_box_size_x", data.getCollisionBoxSizeX());
      ser.write_type_6("collision_box_ground_clearance", data.getCollisionBoxGroundClearance());
      ser.write_type_6("smoother_collision_weight", data.getSmootherCollisionWeight());
      ser.write_type_6("smoother_smoothness_weight", data.getSmootherSmoothnessWeight());
      ser.write_type_6("smoother_turn_point_smoothness_discount", data.getSmootherTurnPointSmoothnessDiscount());
      ser.write_type_6("smoother_min_curvature_to_penalize", data.getSmootherMinCurvatureToPenalize());
      ser.write_type_6("smoother_equal_spacing_weight", data.getSmootherEqualSpacingWeight());
      ser.write_type_6("smoother_displacement_weight", data.getSmootherDisplacementWeight());
      ser.write_type_6("smoother_hill_climb_gain", data.getSmootherHillClimbGain());
      ser.write_type_6("smoother_gradient_threshold_to_terminate", data.getSmootherGradientThresholdToTerminate());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setCheckForCollisions(ser.read_type_7("check_for_collisions"));
      data.setPerformSmoothing(ser.read_type_7("perform_smoothing"));
      data.setSnapRadius(ser.read_type_6("snap_radius"));
      data.setMinSnapHeightThreshold(ser.read_type_6("min_snap_height_threshold"));
      data.setInclineCostWeight(ser.read_type_6("incline_cost_weight"));
      data.setInclineCostDeadband(ser.read_type_6("incline_cost_deadband"));
      data.setMaxIncline(ser.read_type_6("max_incline"));
      data.setCollisionBoxSizeY(ser.read_type_6("collision_box_size_y"));
      data.setCollisionBoxSizeX(ser.read_type_6("collision_box_size_x"));
      data.setCollisionBoxGroundClearance(ser.read_type_6("collision_box_ground_clearance"));
      data.setSmootherCollisionWeight(ser.read_type_6("smoother_collision_weight"));
      data.setSmootherSmoothnessWeight(ser.read_type_6("smoother_smoothness_weight"));
      data.setSmootherTurnPointSmoothnessDiscount(ser.read_type_6("smoother_turn_point_smoothness_discount"));
      data.setSmootherMinCurvatureToPenalize(ser.read_type_6("smoother_min_curvature_to_penalize"));
      data.setSmootherEqualSpacingWeight(ser.read_type_6("smoother_equal_spacing_weight"));
      data.setSmootherDisplacementWeight(ser.read_type_6("smoother_displacement_weight"));
      data.setSmootherHillClimbGain(ser.read_type_6("smoother_hill_climb_gain"));
      data.setSmootherGradientThresholdToTerminate(ser.read_type_6("smoother_gradient_threshold_to_terminate"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket src, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket createData()
   {
      return new toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket src, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AStarBodyPathPlannerParametersPacketPubSubType newInstance()
   {
      return new AStarBodyPathPlannerParametersPacketPubSubType();
   }
}
