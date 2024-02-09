package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8ICPNodeMessage" defined in "YOLOv8ICPNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8ICPNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8ICPNodeMessage_.idl instead.
*
*/
public class YOLOv8ICPNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8ICPNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8ICPNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d7a1a479cb8bac09e293fb197bd10dd9b15d0331258b02d4e12cf669a26b0a47";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8ICPNodeMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      cdr.write_type_2(data.getMaskErosionKernelRadius());

      cdr.write_type_6(data.getOutlierFilterThreshold());

      cdr.write_type_2(data.getIcpIterations());

      cdr.write_type_6(data.getBaseDistanceThreshold());

      cdr.write_type_7(data.getRunIcp());

      cdr.write_type_6(data.getMovementDistanceThreshold());

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      data.setMaskErosionKernelRadius(cdr.read_type_2());
      	
      data.setOutlierFilterThreshold(cdr.read_type_6());
      	
      data.setIcpIterations(cdr.read_type_2());
      	
      data.setBaseDistanceThreshold(cdr.read_type_6());
      	
      data.setRunIcp(cdr.read_type_7());
      	
      data.setMovementDistanceThreshold(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_2("mask_erosion_kernel_radius", data.getMaskErosionKernelRadius());
      ser.write_type_6("outlier_filter_threshold", data.getOutlierFilterThreshold());
      ser.write_type_2("icp_iterations", data.getIcpIterations());
      ser.write_type_6("base_distance_threshold", data.getBaseDistanceThreshold());
      ser.write_type_7("run_icp", data.getRunIcp());
      ser.write_type_6("movement_distance_threshold", data.getMovementDistanceThreshold());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ICPNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      data.setMaskErosionKernelRadius(ser.read_type_2("mask_erosion_kernel_radius"));
      data.setOutlierFilterThreshold(ser.read_type_6("outlier_filter_threshold"));
      data.setIcpIterations(ser.read_type_2("icp_iterations"));
      data.setBaseDistanceThreshold(ser.read_type_6("base_distance_threshold"));
      data.setRunIcp(ser.read_type_7("run_icp"));
      data.setMovementDistanceThreshold(ser.read_type_6("movement_distance_threshold"));
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8ICPNodeMessage src, perception_msgs.msg.dds.YOLOv8ICPNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8ICPNodeMessage createData()
   {
      return new perception_msgs.msg.dds.YOLOv8ICPNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8ICPNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8ICPNodeMessage src, perception_msgs.msg.dds.YOLOv8ICPNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8ICPNodeMessagePubSubType newInstance()
   {
      return new YOLOv8ICPNodeMessagePubSubType();
   }
}
