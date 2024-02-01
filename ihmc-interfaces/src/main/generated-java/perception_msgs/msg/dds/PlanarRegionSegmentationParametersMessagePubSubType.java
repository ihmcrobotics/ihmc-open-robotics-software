package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PlanarRegionSegmentationParametersMessage" defined in "PlanarRegionSegmentationParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PlanarRegionSegmentationParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PlanarRegionSegmentationParametersMessage_.idl instead.
*
*/
public class PlanarRegionSegmentationParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PlanarRegionSegmentationParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "350467b4eb244e4cbfe551640ab387d8ac91f1b3af14cc6f62f40e038b61fc46";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getSearchRadius());

      cdr.write_type_6(data.getMaxDistanceFromPlane());

      cdr.write_type_6(data.getMaxAngleFromPlane());

      cdr.write_type_6(data.getMinNormalQuality());

      cdr.write_type_2(data.getMinRegionSize());

      cdr.write_type_6(data.getMaxStandardDeviation());

      cdr.write_type_6(data.getMinVolumicDensity());

   }

   public static void read(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setSearchRadius(cdr.read_type_6());
      	
      data.setMaxDistanceFromPlane(cdr.read_type_6());
      	
      data.setMaxAngleFromPlane(cdr.read_type_6());
      	
      data.setMinNormalQuality(cdr.read_type_6());
      	
      data.setMinRegionSize(cdr.read_type_2());
      	
      data.setMaxStandardDeviation(cdr.read_type_6());
      	
      data.setMinVolumicDensity(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("search_radius", data.getSearchRadius());
      ser.write_type_6("max_distance_from_plane", data.getMaxDistanceFromPlane());
      ser.write_type_6("max_angle_from_plane", data.getMaxAngleFromPlane());
      ser.write_type_6("min_normal_quality", data.getMinNormalQuality());
      ser.write_type_2("min_region_size", data.getMinRegionSize());
      ser.write_type_6("max_standard_deviation", data.getMaxStandardDeviation());
      ser.write_type_6("min_volumic_density", data.getMinVolumicDensity());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setSearchRadius(ser.read_type_6("search_radius"));
      data.setMaxDistanceFromPlane(ser.read_type_6("max_distance_from_plane"));
      data.setMaxAngleFromPlane(ser.read_type_6("max_angle_from_plane"));
      data.setMinNormalQuality(ser.read_type_6("min_normal_quality"));
      data.setMinRegionSize(ser.read_type_2("min_region_size"));
      data.setMaxStandardDeviation(ser.read_type_6("max_standard_deviation"));
      data.setMinVolumicDensity(ser.read_type_6("min_volumic_density"));
   }

   public static void staticCopy(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage src, perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage createData()
   {
      return new perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage src, perception_msgs.msg.dds.PlanarRegionSegmentationParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PlanarRegionSegmentationParametersMessagePubSubType newInstance()
   {
      return new PlanarRegionSegmentationParametersMessagePubSubType();
   }
}
