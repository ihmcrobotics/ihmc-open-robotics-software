package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedObjectPacket" defined in "DetectedObjectPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedObjectPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedObjectPacket_.idl instead.
*
*/
public class DetectedObjectPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedObjectPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedObjectPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c3629262e54c9df5a5580f9c0bfcd71ff5e4dc255ac5a8f7e341ef0893d1e4fd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedObjectPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedObjectPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      for(int i0 = 0; i0 < (8); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      for(int i0 = 0; i0 < (8); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedObjectPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedObjectPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getSensorPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectType().length() + 1;

      for(int i0 = 0; i0 < data.getBoundingBox2dVertices().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBox2dVertices()[i0], current_alignment);
      }
      for(int i0 = 0; i0 < data.getBoundingBoxVertices().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBoxVertices()[i0], current_alignment);
      }

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedObjectPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getSensorPose(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
      cdr.write_type_6(data.getConfidence());

      if(data.getObjectType().length() <= 255)
      cdr.write_type_d(data.getObjectType());else
          throw new RuntimeException("object_type field exceeds the maximum length");

      for(int i0 = 0; i0 < data.getBoundingBox2dVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBox2dVertices()[i0], cdr);		
      }

      for(int i0 = 0; i0 < data.getBoundingBoxVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBoxVertices()[i0], cdr);		
      }

   }

   public static void read(perception_msgs.msg.dds.DetectedObjectPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setId(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getSensorPose(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	
      data.setConfidence(cdr.read_type_6());
      	
      cdr.read_type_d(data.getObjectType());	
      for(int i0 = 0; i0 < data.getBoundingBox2dVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBox2dVertices()[i0], cdr);	
      }
      	
      for(int i0 = 0; i0 < data.getBoundingBoxVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBoxVertices()[i0], cdr);	
      }
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedObjectPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("id", data.getId());
      ser.write_type_a("sensor_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSensorPose());

      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.write_type_6("confidence", data.getConfidence());
      ser.write_type_d("object_type", data.getObjectType());
      ser.write_type_f("bounding_box_2d_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBox2dVertices());
      ser.write_type_f("bounding_box_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxVertices());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedObjectPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setId(ser.read_type_2("id"));
      ser.read_type_a("sensor_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSensorPose());

      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      data.setConfidence(ser.read_type_6("confidence"));
      ser.read_type_d("object_type", data.getObjectType());
      ser.read_type_f("bounding_box_2d_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBox2dVertices());
      ser.read_type_f("bounding_box_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxVertices());
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedObjectPacket src, perception_msgs.msg.dds.DetectedObjectPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedObjectPacket createData()
   {
      return new perception_msgs.msg.dds.DetectedObjectPacket();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedObjectPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedObjectPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedObjectPacket src, perception_msgs.msg.dds.DetectedObjectPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedObjectPacketPubSubType newInstance()
   {
      return new DetectedObjectPacketPubSubType();
   }
}
