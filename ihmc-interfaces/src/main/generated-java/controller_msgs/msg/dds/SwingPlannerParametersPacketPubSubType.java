package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SwingPlannerParametersPacket" defined in "SwingPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SwingPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SwingPlannerParametersPacket_.idl instead.
*
*/
public class SwingPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SwingPlannerParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SwingPlannerParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SwingPlannerParametersPacket data) throws java.io.IOException
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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SwingPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SwingPlannerParametersPacket data, int current_alignment)
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



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
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


      cdr.write_type_6(data.getMinimumSwingHeight());


      cdr.write_type_6(data.getMaximumSwingHeight());


      cdr.write_type_6(data.getMaximumStepHeightForMinimumSwingHeight());


      cdr.write_type_6(data.getMinimumStepHeightForMaximumSwingHeight());


      cdr.write_type_6(data.getMinimumSwingTime());


      cdr.write_type_6(data.getMaximumSwingTime());


      cdr.write_type_6(data.getMaximumStepTranslationForMinimumSwingTime());


      cdr.write_type_6(data.getMinimumStepTranslationForMaximumSwingTime());


      cdr.write_type_6(data.getMaximumStepHeightForMinimumSwingTime());


      cdr.write_type_6(data.getMinimumStepHeightForMaximumSwingTime());


      cdr.write_type_6(data.getFootStubClearance());


      cdr.write_type_6(data.getWaypointProportionShiftForStubAvoidance());


      cdr.write_type_6(data.getAdditionalSwingTimeIfExpanded());

   }

   public static void read(controller_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
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
      	

      data.setMinimumSwingHeight(cdr.read_type_6());
      	

      data.setMaximumSwingHeight(cdr.read_type_6());
      	

      data.setMaximumStepHeightForMinimumSwingHeight(cdr.read_type_6());
      	

      data.setMinimumStepHeightForMaximumSwingHeight(cdr.read_type_6());
      	

      data.setMinimumSwingTime(cdr.read_type_6());
      	

      data.setMaximumSwingTime(cdr.read_type_6());
      	

      data.setMaximumStepTranslationForMinimumSwingTime(cdr.read_type_6());
      	

      data.setMinimumStepTranslationForMaximumSwingTime(cdr.read_type_6());
      	

      data.setMaximumStepHeightForMinimumSwingTime(cdr.read_type_6());
      	

      data.setMinimumStepHeightForMaximumSwingTime(cdr.read_type_6());
      	

      data.setFootStubClearance(cdr.read_type_6());
      	

      data.setWaypointProportionShiftForStubAvoidance(cdr.read_type_6());
      	

      data.setAdditionalSwingTimeIfExpanded(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
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

      ser.write_type_6("minimum_swing_height", data.getMinimumSwingHeight());

      ser.write_type_6("maximum_swing_height", data.getMaximumSwingHeight());

      ser.write_type_6("maximum_step_height_for_minimum_swing_height", data.getMaximumStepHeightForMinimumSwingHeight());

      ser.write_type_6("minimum_step_height_for_maximum_swing_height", data.getMinimumStepHeightForMaximumSwingHeight());

      ser.write_type_6("minimum_swing_time", data.getMinimumSwingTime());

      ser.write_type_6("maximum_swing_time", data.getMaximumSwingTime());

      ser.write_type_6("maximum_step_translation_for_minimum_swing_time", data.getMaximumStepTranslationForMinimumSwingTime());

      ser.write_type_6("minimum_step_translation_for_maximum_swing_time", data.getMinimumStepTranslationForMaximumSwingTime());

      ser.write_type_6("maximum_step_height_for_minimum_swing_time", data.getMaximumStepHeightForMinimumSwingTime());

      ser.write_type_6("minimum_step_height_for_maximum_swing_time", data.getMinimumStepHeightForMaximumSwingTime());

      ser.write_type_6("foot_stub_clearance", data.getFootStubClearance());

      ser.write_type_6("waypoint_proportion_shift_for_stub_avoidance", data.getWaypointProportionShiftForStubAvoidance());

      ser.write_type_6("additional_swing_time_if_expanded", data.getAdditionalSwingTimeIfExpanded());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SwingPlannerParametersPacket data)
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

      data.setMinimumSwingHeight(ser.read_type_6("minimum_swing_height"));

      data.setMaximumSwingHeight(ser.read_type_6("maximum_swing_height"));

      data.setMaximumStepHeightForMinimumSwingHeight(ser.read_type_6("maximum_step_height_for_minimum_swing_height"));

      data.setMinimumStepHeightForMaximumSwingHeight(ser.read_type_6("minimum_step_height_for_maximum_swing_height"));

      data.setMinimumSwingTime(ser.read_type_6("minimum_swing_time"));

      data.setMaximumSwingTime(ser.read_type_6("maximum_swing_time"));

      data.setMaximumStepTranslationForMinimumSwingTime(ser.read_type_6("maximum_step_translation_for_minimum_swing_time"));

      data.setMinimumStepTranslationForMaximumSwingTime(ser.read_type_6("minimum_step_translation_for_maximum_swing_time"));

      data.setMaximumStepHeightForMinimumSwingTime(ser.read_type_6("maximum_step_height_for_minimum_swing_time"));

      data.setMinimumStepHeightForMaximumSwingTime(ser.read_type_6("minimum_step_height_for_maximum_swing_time"));

      data.setFootStubClearance(ser.read_type_6("foot_stub_clearance"));

      data.setWaypointProportionShiftForStubAvoidance(ser.read_type_6("waypoint_proportion_shift_for_stub_avoidance"));

      data.setAdditionalSwingTimeIfExpanded(ser.read_type_6("additional_swing_time_if_expanded"));
   }

   public static void staticCopy(controller_msgs.msg.dds.SwingPlannerParametersPacket src, controller_msgs.msg.dds.SwingPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SwingPlannerParametersPacket createData()
   {
      return new controller_msgs.msg.dds.SwingPlannerParametersPacket();
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
   
   public void serialize(controller_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SwingPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SwingPlannerParametersPacket src, controller_msgs.msg.dds.SwingPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SwingPlannerParametersPacketPubSubType newInstance()
   {
      return new SwingPlannerParametersPacketPubSubType();
   }
}
