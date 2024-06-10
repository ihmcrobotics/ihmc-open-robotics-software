package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8NodeMessage" defined in "YOLOv8NodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8NodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8NodeMessage_.idl instead.
*
*/
public class YOLOv8NodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8NodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8NodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bb4cf9c215fb9b6f46836877987811189a2a4b4b6c0d13b56fee926975125a66";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8NodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8NodeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 5000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8NodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8NodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDetectionClass().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getObjectPointCloud().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getCdrSerializedSize(data.getObjectPointCloud().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getCdrSerializedSize(data.getObjectCentroid(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getCdrSerializedSize(data.getCentroidToObjectTransform(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getFilteredObjectPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8NodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      cdr.write_type_5(data.getDetectionAcceptanceThreshold());

      if(data.getDetectionClass().length() <= 255)
      cdr.write_type_d(data.getDetectionClass());else
          throw new RuntimeException("detection_class field exceeds the maximum length");

      cdr.write_type_5(data.getConfidence());

      cdr.write_type_2(data.getX());

      cdr.write_type_2(data.getY());

      cdr.write_type_2(data.getWidth());

      cdr.write_type_2(data.getHeight());

      if(data.getObjectPointCloud().size() <= 5000)
      cdr.write_type_e(data.getObjectPointCloud());else
          throw new RuntimeException("object_point_cloud field exceeds the maximum length");

      geometry_msgs.msg.dds.Point32PubSubType.write(data.getObjectCentroid(), cdr);
      geometry_msgs.msg.dds.TransformPubSubType.write(data.getCentroidToObjectTransform(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getFilteredObjectPose(), cdr);
   }

   public static void read(perception_msgs.msg.dds.YOLOv8NodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      data.setDetectionAcceptanceThreshold(cdr.read_type_5());
      	
      cdr.read_type_d(data.getDetectionClass());	
      data.setConfidence(cdr.read_type_5());
      	
      data.setX(cdr.read_type_2());
      	
      data.setY(cdr.read_type_2());
      	
      data.setWidth(cdr.read_type_2());
      	
      data.setHeight(cdr.read_type_2());
      	
      cdr.read_type_e(data.getObjectPointCloud());	
      geometry_msgs.msg.dds.Point32PubSubType.read(data.getObjectCentroid(), cdr);	
      geometry_msgs.msg.dds.TransformPubSubType.read(data.getCentroidToObjectTransform(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getFilteredObjectPose(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8NodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_5("detection_acceptance_threshold", data.getDetectionAcceptanceThreshold());
      ser.write_type_d("detection_class", data.getDetectionClass());
      ser.write_type_5("confidence", data.getConfidence());
      ser.write_type_2("x", data.getX());
      ser.write_type_2("y", data.getY());
      ser.write_type_2("width", data.getWidth());
      ser.write_type_2("height", data.getHeight());
      ser.write_type_e("object_point_cloud", data.getObjectPointCloud());
      ser.write_type_a("object_centroid", new geometry_msgs.msg.dds.Point32PubSubType(), data.getObjectCentroid());

      ser.write_type_a("centroid_to_object_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getCentroidToObjectTransform());

      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.write_type_a("filtered_object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getFilteredObjectPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8NodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      data.setDetectionAcceptanceThreshold(ser.read_type_5("detection_acceptance_threshold"));
      ser.read_type_d("detection_class", data.getDetectionClass());
      data.setConfidence(ser.read_type_5("confidence"));
      data.setX(ser.read_type_2("x"));
      data.setY(ser.read_type_2("y"));
      data.setWidth(ser.read_type_2("width"));
      data.setHeight(ser.read_type_2("height"));
      ser.read_type_e("object_point_cloud", data.getObjectPointCloud());
      ser.read_type_a("object_centroid", new geometry_msgs.msg.dds.Point32PubSubType(), data.getObjectCentroid());

      ser.read_type_a("centroid_to_object_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getCentroidToObjectTransform());

      ser.read_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.read_type_a("filtered_object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getFilteredObjectPose());

   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8NodeMessage src, perception_msgs.msg.dds.YOLOv8NodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8NodeMessage createData()
   {
      return new perception_msgs.msg.dds.YOLOv8NodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8NodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8NodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8NodeMessage src, perception_msgs.msg.dds.YOLOv8NodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8NodeMessagePubSubType newInstance()
   {
      return new YOLOv8NodeMessagePubSubType();
   }
}
