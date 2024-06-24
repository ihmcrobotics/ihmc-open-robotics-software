package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "InstantDetectionMessage" defined in "InstantDetectionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from InstantDetectionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit InstantDetectionMessage_.idl instead.
*
*/
public class InstantDetectionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.InstantDetectionMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::InstantDetectionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "af750cbd004ae4bfc64172f67f7a4e6628326bea9351a7faf466eec0e6e6d2f9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.InstantDetectionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.InstantDetectionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 511; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getMaxCdrSerializedSize(current_alignment);}
      for(int i0 = 0; i0 < (8); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      for(int i0 = 0; i0 < (8); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.InstantDetectionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.InstantDetectionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDetectedObjectClass().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDetectedObjectName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getDetectionTime(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.getCdrSerializedSize(data.getPersistentDetectionId(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getYoloObjectPointCloud().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getCdrSerializedSize(data.getYoloObjectPointCloud().get(i0), current_alignment);}

      for(int i0 = 0; i0 < data.getCenterPoseBoundingBox2dVertices().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCenterPoseBoundingBox2dVertices()[i0], current_alignment);
      }
      for(int i0 = 0; i0 < data.getCenterPoseBoundingBoxVertices().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCenterPoseBoundingBoxVertices()[i0], current_alignment);
      }

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.InstantDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDetectedObjectClass().length() <= 255)
      cdr.write_type_d(data.getDetectedObjectClass());else
          throw new RuntimeException("detected_object_class field exceeds the maximum length");

      if(data.getDetectedObjectName().length() <= 255)
      cdr.write_type_d(data.getDetectedObjectName());else
          throw new RuntimeException("detected_object_name field exceeds the maximum length");

      cdr.write_type_6(data.getConfidence());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getDetectionTime(), cdr);
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.write(data.getPersistentDetectionId(), cdr);
      if(data.getYoloObjectPointCloud().size() <= 511)
      cdr.write_type_e(data.getYoloObjectPointCloud());else
          throw new RuntimeException("yolo_object_point_cloud field exceeds the maximum length");

      for(int i0 = 0; i0 < data.getCenterPoseBoundingBox2dVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.write(data.getCenterPoseBoundingBox2dVertices()[i0], cdr);		
      }

      for(int i0 = 0; i0 < data.getCenterPoseBoundingBoxVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.write(data.getCenterPoseBoundingBoxVertices()[i0], cdr);		
      }

   }

   public static void read(perception_msgs.msg.dds.InstantDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getDetectedObjectClass());	
      cdr.read_type_d(data.getDetectedObjectName());	
      data.setConfidence(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getDetectionTime(), cdr);	
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.read(data.getPersistentDetectionId(), cdr);	
      cdr.read_type_e(data.getYoloObjectPointCloud());	
      for(int i0 = 0; i0 < data.getCenterPoseBoundingBox2dVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.read(data.getCenterPoseBoundingBox2dVertices()[i0], cdr);	
      }
      	
      for(int i0 = 0; i0 < data.getCenterPoseBoundingBoxVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.read(data.getCenterPoseBoundingBoxVertices()[i0], cdr);	
      }
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.InstantDetectionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("detected_object_class", data.getDetectedObjectClass());
      ser.write_type_d("detected_object_name", data.getDetectedObjectName());
      ser.write_type_6("confidence", data.getConfidence());
      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.write_type_a("detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionTime());

      ser.write_type_a("persistent_detection_id", new ihmc_common_msgs.msg.dds.UUIDMessagePubSubType(), data.getPersistentDetectionId());

      ser.write_type_e("yolo_object_point_cloud", data.getYoloObjectPointCloud());
      ser.write_type_f("center_pose_bounding_box_2d_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPoseBoundingBox2dVertices());
      ser.write_type_f("center_pose_bounding_box_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPoseBoundingBoxVertices());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.InstantDetectionMessage data)
   {
      ser.read_type_d("detected_object_class", data.getDetectedObjectClass());
      ser.read_type_d("detected_object_name", data.getDetectedObjectName());
      data.setConfidence(ser.read_type_6("confidence"));
      ser.read_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.read_type_a("detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionTime());

      ser.read_type_a("persistent_detection_id", new ihmc_common_msgs.msg.dds.UUIDMessagePubSubType(), data.getPersistentDetectionId());

      ser.read_type_e("yolo_object_point_cloud", data.getYoloObjectPointCloud());
      ser.read_type_f("center_pose_bounding_box_2d_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPoseBoundingBox2dVertices());
      ser.read_type_f("center_pose_bounding_box_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPoseBoundingBoxVertices());
   }

   public static void staticCopy(perception_msgs.msg.dds.InstantDetectionMessage src, perception_msgs.msg.dds.InstantDetectionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.InstantDetectionMessage createData()
   {
      return new perception_msgs.msg.dds.InstantDetectionMessage();
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
   
   public void serialize(perception_msgs.msg.dds.InstantDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.InstantDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.InstantDetectionMessage src, perception_msgs.msg.dds.InstantDetectionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public InstantDetectionMessagePubSubType newInstance()
   {
      return new InstantDetectionMessagePubSubType();
   }
}
