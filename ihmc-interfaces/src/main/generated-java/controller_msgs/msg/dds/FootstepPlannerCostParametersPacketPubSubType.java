package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerCostParametersPacket" defined in "FootstepPlannerCostParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerCostParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerCostParametersPacket_.idl instead.
*
*/
public class FootstepPlannerCostParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerCostParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerCostParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, int current_alignment)
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


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getYawWeight());

      cdr.write_type_6(data.getPitchWeight());

      cdr.write_type_6(data.getRollWeight());

      cdr.write_type_6(data.getForwardWeight());

      cdr.write_type_6(data.getLateralWeight());

      cdr.write_type_6(data.getStepUpWeight());

      cdr.write_type_6(data.getStepDownWeight());

      cdr.write_type_6(data.getCostPerStep());

      cdr.write_type_7(data.getUseQuadraticDistanceCost());

      cdr.write_type_7(data.getUseQuadraticHeightCost());

      cdr.write_type_6(data.getAStarHeuristicsWeight());

      cdr.write_type_6(data.getVisGraphWithAStarHeuristicsWeight());

      cdr.write_type_6(data.getDepthFirstHeuristicsWeight());

      cdr.write_type_6(data.getBodyPathBasedHeuristicsWeight());

      cdr.write_type_6(data.getMaximum2dDistanceFromBoundingBoxToPenalize());

      cdr.write_type_6(data.getBoundingBoxCost());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setYawWeight(cdr.read_type_6());
      	
      data.setPitchWeight(cdr.read_type_6());
      	
      data.setRollWeight(cdr.read_type_6());
      	
      data.setForwardWeight(cdr.read_type_6());
      	
      data.setLateralWeight(cdr.read_type_6());
      	
      data.setStepUpWeight(cdr.read_type_6());
      	
      data.setStepDownWeight(cdr.read_type_6());
      	
      data.setCostPerStep(cdr.read_type_6());
      	
      data.setUseQuadraticDistanceCost(cdr.read_type_7());
      	
      data.setUseQuadraticHeightCost(cdr.read_type_7());
      	
      data.setAStarHeuristicsWeight(cdr.read_type_6());
      	
      data.setVisGraphWithAStarHeuristicsWeight(cdr.read_type_6());
      	
      data.setDepthFirstHeuristicsWeight(cdr.read_type_6());
      	
      data.setBodyPathBasedHeuristicsWeight(cdr.read_type_6());
      	
      data.setMaximum2dDistanceFromBoundingBoxToPenalize(cdr.read_type_6());
      	
      data.setBoundingBoxCost(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("yaw_weight", data.getYawWeight());
      ser.write_type_6("pitch_weight", data.getPitchWeight());
      ser.write_type_6("roll_weight", data.getRollWeight());
      ser.write_type_6("forward_weight", data.getForwardWeight());
      ser.write_type_6("lateral_weight", data.getLateralWeight());
      ser.write_type_6("step_up_weight", data.getStepUpWeight());
      ser.write_type_6("step_down_weight", data.getStepDownWeight());
      ser.write_type_6("cost_per_step", data.getCostPerStep());
      ser.write_type_7("use_quadratic_distance_cost", data.getUseQuadraticDistanceCost());
      ser.write_type_7("use_quadratic_height_cost", data.getUseQuadraticHeightCost());
      ser.write_type_6("a_star_heuristics_weight", data.getAStarHeuristicsWeight());
      ser.write_type_6("vis_graph_with_a_star_heuristics_weight", data.getVisGraphWithAStarHeuristicsWeight());
      ser.write_type_6("depth_first_heuristics_weight", data.getDepthFirstHeuristicsWeight());
      ser.write_type_6("body_path_based_heuristics_weight", data.getBodyPathBasedHeuristicsWeight());
      ser.write_type_6("maximum_2d_distance_from_bounding_box_to_penalize", data.getMaximum2dDistanceFromBoundingBoxToPenalize());
      ser.write_type_6("bounding_box_cost", data.getBoundingBoxCost());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setYawWeight(ser.read_type_6("yaw_weight"));
      data.setPitchWeight(ser.read_type_6("pitch_weight"));
      data.setRollWeight(ser.read_type_6("roll_weight"));
      data.setForwardWeight(ser.read_type_6("forward_weight"));
      data.setLateralWeight(ser.read_type_6("lateral_weight"));
      data.setStepUpWeight(ser.read_type_6("step_up_weight"));
      data.setStepDownWeight(ser.read_type_6("step_down_weight"));
      data.setCostPerStep(ser.read_type_6("cost_per_step"));
      data.setUseQuadraticDistanceCost(ser.read_type_7("use_quadratic_distance_cost"));
      data.setUseQuadraticHeightCost(ser.read_type_7("use_quadratic_height_cost"));
      data.setAStarHeuristicsWeight(ser.read_type_6("a_star_heuristics_weight"));
      data.setVisGraphWithAStarHeuristicsWeight(ser.read_type_6("vis_graph_with_a_star_heuristics_weight"));
      data.setDepthFirstHeuristicsWeight(ser.read_type_6("depth_first_heuristics_weight"));
      data.setBodyPathBasedHeuristicsWeight(ser.read_type_6("body_path_based_heuristics_weight"));
      data.setMaximum2dDistanceFromBoundingBoxToPenalize(ser.read_type_6("maximum_2d_distance_from_bounding_box_to_penalize"));
      data.setBoundingBoxCost(ser.read_type_6("bounding_box_cost"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket src, controller_msgs.msg.dds.FootstepPlannerCostParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerCostParametersPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerCostParametersPacket();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerCostParametersPacket src, controller_msgs.msg.dds.FootstepPlannerCostParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerCostParametersPacketPubSubType newInstance()
   {
      return new FootstepPlannerCostParametersPacketPubSubType();
   }
}
