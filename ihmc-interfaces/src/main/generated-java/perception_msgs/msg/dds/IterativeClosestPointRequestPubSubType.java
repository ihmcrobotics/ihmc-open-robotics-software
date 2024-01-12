package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "IterativeClosestPointRequest" defined in "IterativeClosestPointRequest_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from IterativeClosestPointRequest_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit IterativeClosestPointRequest_.idl instead.
*
*/
public class IterativeClosestPointRequestPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.IterativeClosestPointRequest>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::IterativeClosestPointRequest_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "77db85bc8ba4c24e96daf2a8f5925b75f998efec26b000fa05eff7e7432eba9d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.IterativeClosestPointRequest data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.IterativeClosestPointRequest data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.IterativeClosestPointRequest data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLengths(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRadii(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getProvidedPose(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_4(data.getNodeId());

      cdr.write_type_9(data.getShape());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLengths(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRadii(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getProvidedPose(), cdr);
      cdr.write_type_4(data.getNumberOfShapeSamples());

      cdr.write_type_4(data.getNumberOfCorrespondences());

      cdr.write_type_5(data.getSegmentationRadius());

      cdr.write_type_7(data.getRunIcp());

      cdr.write_type_7(data.getUseProvidedPose());

   }

   public static void read(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setNodeId(cdr.read_type_4());
      	
      data.setShape(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLengths(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRadii(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getProvidedPose(), cdr);	
      data.setNumberOfShapeSamples(cdr.read_type_4());
      	
      data.setNumberOfCorrespondences(cdr.read_type_4());
      	
      data.setSegmentationRadius(cdr.read_type_5());
      	
      data.setRunIcp(cdr.read_type_7());
      	
      data.setUseProvidedPose(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_4("node_id", data.getNodeId());
      ser.write_type_9("shape", data.getShape());
      ser.write_type_a("lengths", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLengths());

      ser.write_type_a("radii", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRadii());

      ser.write_type_a("provided_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getProvidedPose());

      ser.write_type_4("number_of_shape_samples", data.getNumberOfShapeSamples());
      ser.write_type_4("number_of_correspondences", data.getNumberOfCorrespondences());
      ser.write_type_5("segmentation_radius", data.getSegmentationRadius());
      ser.write_type_7("run_icp", data.getRunIcp());
      ser.write_type_7("use_provided_pose", data.getUseProvidedPose());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.IterativeClosestPointRequest data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setNodeId(ser.read_type_4("node_id"));
      data.setShape(ser.read_type_9("shape"));
      ser.read_type_a("lengths", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLengths());

      ser.read_type_a("radii", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRadii());

      ser.read_type_a("provided_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getProvidedPose());

      data.setNumberOfShapeSamples(ser.read_type_4("number_of_shape_samples"));
      data.setNumberOfCorrespondences(ser.read_type_4("number_of_correspondences"));
      data.setSegmentationRadius(ser.read_type_5("segmentation_radius"));
      data.setRunIcp(ser.read_type_7("run_icp"));
      data.setUseProvidedPose(ser.read_type_7("use_provided_pose"));
   }

   public static void staticCopy(perception_msgs.msg.dds.IterativeClosestPointRequest src, perception_msgs.msg.dds.IterativeClosestPointRequest dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.IterativeClosestPointRequest createData()
   {
      return new perception_msgs.msg.dds.IterativeClosestPointRequest();
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
   
   public void serialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.IterativeClosestPointRequest src, perception_msgs.msg.dds.IterativeClosestPointRequest dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public IterativeClosestPointRequestPubSubType newInstance()
   {
      return new IterativeClosestPointRequestPubSubType();
   }
}
