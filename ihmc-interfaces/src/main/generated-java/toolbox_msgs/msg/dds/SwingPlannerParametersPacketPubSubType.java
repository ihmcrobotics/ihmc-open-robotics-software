package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SwingPlannerParametersPacket" defined in "SwingPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SwingPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SwingPlannerParametersPacket_.idl instead.
*
*/
public class SwingPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.SwingPlannerParametersPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::SwingPlannerParametersPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3c31f62d364faeb1464397a1d26b16d46df07b8650817cf84c4968f10c981769";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.SwingPlannerParametersPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getDoInitialFastApproximation());

      cdr.write_type_6(data.getMinimumSwingFootClearance());

      cdr.write_type_6(data.getFastApproximationLessClearance());

      cdr.write_type_4(data.getNumberOfChecksPerSwing());

      cdr.write_type_4(data.getMaximumNumberOfAdjustmentAttempts());

      cdr.write_type_6(data.getMaximumWaypointAdjustmentDistance());

      cdr.write_type_6(data.getMinimumAdjustmentIncrementDistance());

      cdr.write_type_6(data.getMaximumAdjustmentIncrementDistance());

      cdr.write_type_6(data.getAdjustmentIncrementDistanceGain());

      cdr.write_type_6(data.getMinimumHeightAboveFloorForCollision());

      cdr.write_type_6(data.getSwingHeightIfCollisionDetected());

      cdr.write_type_6(data.getMinimumSwingTime());

      cdr.write_type_6(data.getMaximumSwingTime());

      cdr.write_type_6(data.getFootStubClearance());

      cdr.write_type_6(data.getWaypointProportionShiftForStubAvoidance());

      cdr.write_type_6(data.getAdditionalSwingTimeIfExpanded());

      cdr.write_type_7(data.getAllowLateralMotion());

      cdr.write_type_6(data.getMinXyTranslationToPlanSwing());

      cdr.write_type_6(data.getPercentageExtraSizeXLow());

      cdr.write_type_6(data.getPercentageExtraSizeXHigh());

      cdr.write_type_6(data.getExtraSizeXLow());

      cdr.write_type_6(data.getExtraSizeXHigh());

      cdr.write_type_6(data.getPercentageExtraSizeYLow());

      cdr.write_type_6(data.getPercentageExtraSizeYHigh());

      cdr.write_type_6(data.getExtraSizeYLow());

      cdr.write_type_6(data.getExtraSizeYHigh());

      cdr.write_type_6(data.getPercentageExtraSizeZLow());

      cdr.write_type_6(data.getPercentageExtraSizeZHigh());

      cdr.write_type_6(data.getExtraSizeZLow());

      cdr.write_type_6(data.getExtraSizeZHigh());

      cdr.write_type_6(data.getPercentageMaxDisplacementLow());

      cdr.write_type_6(data.getPercentageMaxDisplacementHigh());

      cdr.write_type_6(data.getMaxDisplacementLow());

      cdr.write_type_6(data.getMaxDisplacementHigh());

      cdr.write_type_6(data.getMotionCorrelationAlpha());

   }

   public static void read(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setDoInitialFastApproximation(cdr.read_type_7());
      	
      data.setMinimumSwingFootClearance(cdr.read_type_6());
      	
      data.setFastApproximationLessClearance(cdr.read_type_6());
      	
      data.setNumberOfChecksPerSwing(cdr.read_type_4());
      	
      data.setMaximumNumberOfAdjustmentAttempts(cdr.read_type_4());
      	
      data.setMaximumWaypointAdjustmentDistance(cdr.read_type_6());
      	
      data.setMinimumAdjustmentIncrementDistance(cdr.read_type_6());
      	
      data.setMaximumAdjustmentIncrementDistance(cdr.read_type_6());
      	
      data.setAdjustmentIncrementDistanceGain(cdr.read_type_6());
      	
      data.setMinimumHeightAboveFloorForCollision(cdr.read_type_6());
      	
      data.setSwingHeightIfCollisionDetected(cdr.read_type_6());
      	
      data.setMinimumSwingTime(cdr.read_type_6());
      	
      data.setMaximumSwingTime(cdr.read_type_6());
      	
      data.setFootStubClearance(cdr.read_type_6());
      	
      data.setWaypointProportionShiftForStubAvoidance(cdr.read_type_6());
      	
      data.setAdditionalSwingTimeIfExpanded(cdr.read_type_6());
      	
      data.setAllowLateralMotion(cdr.read_type_7());
      	
      data.setMinXyTranslationToPlanSwing(cdr.read_type_6());
      	
      data.setPercentageExtraSizeXLow(cdr.read_type_6());
      	
      data.setPercentageExtraSizeXHigh(cdr.read_type_6());
      	
      data.setExtraSizeXLow(cdr.read_type_6());
      	
      data.setExtraSizeXHigh(cdr.read_type_6());
      	
      data.setPercentageExtraSizeYLow(cdr.read_type_6());
      	
      data.setPercentageExtraSizeYHigh(cdr.read_type_6());
      	
      data.setExtraSizeYLow(cdr.read_type_6());
      	
      data.setExtraSizeYHigh(cdr.read_type_6());
      	
      data.setPercentageExtraSizeZLow(cdr.read_type_6());
      	
      data.setPercentageExtraSizeZHigh(cdr.read_type_6());
      	
      data.setExtraSizeZLow(cdr.read_type_6());
      	
      data.setExtraSizeZHigh(cdr.read_type_6());
      	
      data.setPercentageMaxDisplacementLow(cdr.read_type_6());
      	
      data.setPercentageMaxDisplacementHigh(cdr.read_type_6());
      	
      data.setMaxDisplacementLow(cdr.read_type_6());
      	
      data.setMaxDisplacementHigh(cdr.read_type_6());
      	
      data.setMotionCorrelationAlpha(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("do_initial_fast_approximation", data.getDoInitialFastApproximation());
      ser.write_type_6("minimum_swing_foot_clearance", data.getMinimumSwingFootClearance());
      ser.write_type_6("fast_approximation_less_clearance", data.getFastApproximationLessClearance());
      ser.write_type_4("number_of_checks_per_swing", data.getNumberOfChecksPerSwing());
      ser.write_type_4("maximum_number_of_adjustment_attempts", data.getMaximumNumberOfAdjustmentAttempts());
      ser.write_type_6("maximum_waypoint_adjustment_distance", data.getMaximumWaypointAdjustmentDistance());
      ser.write_type_6("minimum_adjustment_increment_distance", data.getMinimumAdjustmentIncrementDistance());
      ser.write_type_6("maximum_adjustment_increment_distance", data.getMaximumAdjustmentIncrementDistance());
      ser.write_type_6("adjustment_increment_distance_gain", data.getAdjustmentIncrementDistanceGain());
      ser.write_type_6("minimum_height_above_floor_for_collision", data.getMinimumHeightAboveFloorForCollision());
      ser.write_type_6("swing_height_if_collision_detected", data.getSwingHeightIfCollisionDetected());
      ser.write_type_6("minimum_swing_time", data.getMinimumSwingTime());
      ser.write_type_6("maximum_swing_time", data.getMaximumSwingTime());
      ser.write_type_6("foot_stub_clearance", data.getFootStubClearance());
      ser.write_type_6("waypoint_proportion_shift_for_stub_avoidance", data.getWaypointProportionShiftForStubAvoidance());
      ser.write_type_6("additional_swing_time_if_expanded", data.getAdditionalSwingTimeIfExpanded());
      ser.write_type_7("allow_lateral_motion", data.getAllowLateralMotion());
      ser.write_type_6("min_xy_translation_to_plan_swing", data.getMinXyTranslationToPlanSwing());
      ser.write_type_6("percentage_extra_size_x_low", data.getPercentageExtraSizeXLow());
      ser.write_type_6("percentage_extra_size_x_high", data.getPercentageExtraSizeXHigh());
      ser.write_type_6("extra_size_x_low", data.getExtraSizeXLow());
      ser.write_type_6("extra_size_x_high", data.getExtraSizeXHigh());
      ser.write_type_6("percentage_extra_size_y_low", data.getPercentageExtraSizeYLow());
      ser.write_type_6("percentage_extra_size_y_high", data.getPercentageExtraSizeYHigh());
      ser.write_type_6("extra_size_y_low", data.getExtraSizeYLow());
      ser.write_type_6("extra_size_y_high", data.getExtraSizeYHigh());
      ser.write_type_6("percentage_extra_size_z_low", data.getPercentageExtraSizeZLow());
      ser.write_type_6("percentage_extra_size_z_high", data.getPercentageExtraSizeZHigh());
      ser.write_type_6("extra_size_z_low", data.getExtraSizeZLow());
      ser.write_type_6("extra_size_z_high", data.getExtraSizeZHigh());
      ser.write_type_6("percentage_max_displacement_low", data.getPercentageMaxDisplacementLow());
      ser.write_type_6("percentage_max_displacement_high", data.getPercentageMaxDisplacementHigh());
      ser.write_type_6("max_displacement_low", data.getMaxDisplacementLow());
      ser.write_type_6("max_displacement_high", data.getMaxDisplacementHigh());
      ser.write_type_6("motion_correlation_alpha", data.getMotionCorrelationAlpha());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.SwingPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setDoInitialFastApproximation(ser.read_type_7("do_initial_fast_approximation"));
      data.setMinimumSwingFootClearance(ser.read_type_6("minimum_swing_foot_clearance"));
      data.setFastApproximationLessClearance(ser.read_type_6("fast_approximation_less_clearance"));
      data.setNumberOfChecksPerSwing(ser.read_type_4("number_of_checks_per_swing"));
      data.setMaximumNumberOfAdjustmentAttempts(ser.read_type_4("maximum_number_of_adjustment_attempts"));
      data.setMaximumWaypointAdjustmentDistance(ser.read_type_6("maximum_waypoint_adjustment_distance"));
      data.setMinimumAdjustmentIncrementDistance(ser.read_type_6("minimum_adjustment_increment_distance"));
      data.setMaximumAdjustmentIncrementDistance(ser.read_type_6("maximum_adjustment_increment_distance"));
      data.setAdjustmentIncrementDistanceGain(ser.read_type_6("adjustment_increment_distance_gain"));
      data.setMinimumHeightAboveFloorForCollision(ser.read_type_6("minimum_height_above_floor_for_collision"));
      data.setSwingHeightIfCollisionDetected(ser.read_type_6("swing_height_if_collision_detected"));
      data.setMinimumSwingTime(ser.read_type_6("minimum_swing_time"));
      data.setMaximumSwingTime(ser.read_type_6("maximum_swing_time"));
      data.setFootStubClearance(ser.read_type_6("foot_stub_clearance"));
      data.setWaypointProportionShiftForStubAvoidance(ser.read_type_6("waypoint_proportion_shift_for_stub_avoidance"));
      data.setAdditionalSwingTimeIfExpanded(ser.read_type_6("additional_swing_time_if_expanded"));
      data.setAllowLateralMotion(ser.read_type_7("allow_lateral_motion"));
      data.setMinXyTranslationToPlanSwing(ser.read_type_6("min_xy_translation_to_plan_swing"));
      data.setPercentageExtraSizeXLow(ser.read_type_6("percentage_extra_size_x_low"));
      data.setPercentageExtraSizeXHigh(ser.read_type_6("percentage_extra_size_x_high"));
      data.setExtraSizeXLow(ser.read_type_6("extra_size_x_low"));
      data.setExtraSizeXHigh(ser.read_type_6("extra_size_x_high"));
      data.setPercentageExtraSizeYLow(ser.read_type_6("percentage_extra_size_y_low"));
      data.setPercentageExtraSizeYHigh(ser.read_type_6("percentage_extra_size_y_high"));
      data.setExtraSizeYLow(ser.read_type_6("extra_size_y_low"));
      data.setExtraSizeYHigh(ser.read_type_6("extra_size_y_high"));
      data.setPercentageExtraSizeZLow(ser.read_type_6("percentage_extra_size_z_low"));
      data.setPercentageExtraSizeZHigh(ser.read_type_6("percentage_extra_size_z_high"));
      data.setExtraSizeZLow(ser.read_type_6("extra_size_z_low"));
      data.setExtraSizeZHigh(ser.read_type_6("extra_size_z_high"));
      data.setPercentageMaxDisplacementLow(ser.read_type_6("percentage_max_displacement_low"));
      data.setPercentageMaxDisplacementHigh(ser.read_type_6("percentage_max_displacement_high"));
      data.setMaxDisplacementLow(ser.read_type_6("max_displacement_low"));
      data.setMaxDisplacementHigh(ser.read_type_6("max_displacement_high"));
      data.setMotionCorrelationAlpha(ser.read_type_6("motion_correlation_alpha"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.SwingPlannerParametersPacket src, toolbox_msgs.msg.dds.SwingPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.SwingPlannerParametersPacket createData()
   {
      return new toolbox_msgs.msg.dds.SwingPlannerParametersPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.SwingPlannerParametersPacket src, toolbox_msgs.msg.dds.SwingPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SwingPlannerParametersPacketPubSubType newInstance()
   {
      return new SwingPlannerParametersPacketPubSubType();
   }
}
