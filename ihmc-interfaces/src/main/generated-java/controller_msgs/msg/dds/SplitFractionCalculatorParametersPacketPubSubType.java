package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SplitFractionCalculatorParametersPacket" defined in "SplitFractionCalculatorParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SplitFractionCalculatorParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SplitFractionCalculatorParametersPacket_.idl instead.
*
*/
public class SplitFractionCalculatorParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SplitFractionCalculatorParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, int current_alignment)
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



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_6(data.getDefaultTransferSplitFraction());


      cdr.write_type_6(data.getStepHeightForLargeStepDown());


      cdr.write_type_6(data.getLargestStepDownHeight());


      cdr.write_type_6(data.getTransferSplitFractionAtFullDepth());


      cdr.write_type_6(data.getTransferWeightDistributionAtFullDepth());


      cdr.write_type_6(data.getFractionLoadIfFootHasFullSupport());


      cdr.write_type_6(data.getFractionTimeOnFootIfFootHasFullSupport());


      cdr.write_type_6(data.getFractionLoadIfOtherFootHasNoWidth());


      cdr.write_type_6(data.getFractionTimeOnFootIfOtherFootHasNoWidth());

   }

   public static void read(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setDefaultTransferSplitFraction(cdr.read_type_6());
      	

      data.setStepHeightForLargeStepDown(cdr.read_type_6());
      	

      data.setLargestStepDownHeight(cdr.read_type_6());
      	

      data.setTransferSplitFractionAtFullDepth(cdr.read_type_6());
      	

      data.setTransferWeightDistributionAtFullDepth(cdr.read_type_6());
      	

      data.setFractionLoadIfFootHasFullSupport(cdr.read_type_6());
      	

      data.setFractionTimeOnFootIfFootHasFullSupport(cdr.read_type_6());
      	

      data.setFractionLoadIfOtherFootHasNoWidth(cdr.read_type_6());
      	

      data.setFractionTimeOnFootIfOtherFootHasNoWidth(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_6("default_transfer_split_fraction", data.getDefaultTransferSplitFraction());

      ser.write_type_6("step_height_for_large_step_down", data.getStepHeightForLargeStepDown());

      ser.write_type_6("largest_step_down_height", data.getLargestStepDownHeight());

      ser.write_type_6("transfer_split_fraction_at_full_depth", data.getTransferSplitFractionAtFullDepth());

      ser.write_type_6("transfer_weight_distribution_at_full_depth", data.getTransferWeightDistributionAtFullDepth());

      ser.write_type_6("fraction_load_if_foot_has_full_support", data.getFractionLoadIfFootHasFullSupport());

      ser.write_type_6("fraction_time_on_foot_if_foot_has_full_support", data.getFractionTimeOnFootIfFootHasFullSupport());

      ser.write_type_6("fraction_load_if_other_foot_has_no_width", data.getFractionLoadIfOtherFootHasNoWidth());

      ser.write_type_6("fraction_time_on_foot_if_other_foot_has_no_width", data.getFractionTimeOnFootIfOtherFootHasNoWidth());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setDefaultTransferSplitFraction(ser.read_type_6("default_transfer_split_fraction"));

      data.setStepHeightForLargeStepDown(ser.read_type_6("step_height_for_large_step_down"));

      data.setLargestStepDownHeight(ser.read_type_6("largest_step_down_height"));

      data.setTransferSplitFractionAtFullDepth(ser.read_type_6("transfer_split_fraction_at_full_depth"));

      data.setTransferWeightDistributionAtFullDepth(ser.read_type_6("transfer_weight_distribution_at_full_depth"));

      data.setFractionLoadIfFootHasFullSupport(ser.read_type_6("fraction_load_if_foot_has_full_support"));

      data.setFractionTimeOnFootIfFootHasFullSupport(ser.read_type_6("fraction_time_on_foot_if_foot_has_full_support"));

      data.setFractionLoadIfOtherFootHasNoWidth(ser.read_type_6("fraction_load_if_other_foot_has_no_width"));

      data.setFractionTimeOnFootIfOtherFootHasNoWidth(ser.read_type_6("fraction_time_on_foot_if_other_foot_has_no_width"));
   }

   public static void staticCopy(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket src, controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket createData()
   {
      return new controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket();
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
   
   public void serialize(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket src, controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SplitFractionCalculatorParametersPacketPubSubType newInstance()
   {
      return new SplitFractionCalculatorParametersPacketPubSubType();
   }
}
