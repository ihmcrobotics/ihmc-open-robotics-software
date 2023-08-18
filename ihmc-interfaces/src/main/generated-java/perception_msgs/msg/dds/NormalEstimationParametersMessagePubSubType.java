package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "NormalEstimationParametersMessage" defined in "NormalEstimationParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from NormalEstimationParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit NormalEstimationParametersMessage_.idl instead.
*
*/
public class NormalEstimationParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.NormalEstimationParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::NormalEstimationParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ea1f75a814e7b67b545de9f643f6574f33cbcb518348389d75c0ba01cb99ffc3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.NormalEstimationParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.NormalEstimationParametersMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.NormalEstimationParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.NormalEstimationParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.NormalEstimationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getSearchRadius());

      cdr.write_type_6(data.getMaxDistanceFromPlane());

      cdr.write_type_6(data.getMinConsensusRatio());

      cdr.write_type_6(data.getMaxAverageDeviationRatio());

      cdr.write_type_2(data.getNumberOfIterations());

      cdr.write_type_7(data.getEnableLeastSquaresEstimation());

      cdr.write_type_7(data.getWeightByNumberOfHits());

   }

   public static void read(perception_msgs.msg.dds.NormalEstimationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setSearchRadius(cdr.read_type_6());
      	
      data.setMaxDistanceFromPlane(cdr.read_type_6());
      	
      data.setMinConsensusRatio(cdr.read_type_6());
      	
      data.setMaxAverageDeviationRatio(cdr.read_type_6());
      	
      data.setNumberOfIterations(cdr.read_type_2());
      	
      data.setEnableLeastSquaresEstimation(cdr.read_type_7());
      	
      data.setWeightByNumberOfHits(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.NormalEstimationParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("search_radius", data.getSearchRadius());
      ser.write_type_6("max_distance_from_plane", data.getMaxDistanceFromPlane());
      ser.write_type_6("min_consensus_ratio", data.getMinConsensusRatio());
      ser.write_type_6("max_average_deviation_ratio", data.getMaxAverageDeviationRatio());
      ser.write_type_2("number_of_iterations", data.getNumberOfIterations());
      ser.write_type_7("enable_least_squares_estimation", data.getEnableLeastSquaresEstimation());
      ser.write_type_7("weight_by_number_of_hits", data.getWeightByNumberOfHits());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.NormalEstimationParametersMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setSearchRadius(ser.read_type_6("search_radius"));
      data.setMaxDistanceFromPlane(ser.read_type_6("max_distance_from_plane"));
      data.setMinConsensusRatio(ser.read_type_6("min_consensus_ratio"));
      data.setMaxAverageDeviationRatio(ser.read_type_6("max_average_deviation_ratio"));
      data.setNumberOfIterations(ser.read_type_2("number_of_iterations"));
      data.setEnableLeastSquaresEstimation(ser.read_type_7("enable_least_squares_estimation"));
      data.setWeightByNumberOfHits(ser.read_type_7("weight_by_number_of_hits"));
   }

   public static void staticCopy(perception_msgs.msg.dds.NormalEstimationParametersMessage src, perception_msgs.msg.dds.NormalEstimationParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.NormalEstimationParametersMessage createData()
   {
      return new perception_msgs.msg.dds.NormalEstimationParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.NormalEstimationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.NormalEstimationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.NormalEstimationParametersMessage src, perception_msgs.msg.dds.NormalEstimationParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public NormalEstimationParametersMessagePubSubType newInstance()
   {
      return new NormalEstimationParametersMessagePubSubType();
   }
}
