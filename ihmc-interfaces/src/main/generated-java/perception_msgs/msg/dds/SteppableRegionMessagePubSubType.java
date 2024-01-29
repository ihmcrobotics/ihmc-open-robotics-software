package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SteppableRegionMessage" defined in "SteppableRegionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SteppableRegionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SteppableRegionMessage_.idl instead.
*
*/
public class SteppableRegionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SteppableRegionMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SteppableRegionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "386e770dadb7d312184610b7a74d9f3aa1a602c2af39fca5cfb0debb87aed143";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SteppableRegionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SteppableRegionMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLastUpdated(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRegionOrigin(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRegionOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRegionNormal(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getVertexBuffer().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getVertexBuffer().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getLocalHeightMap(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SteppableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLastUpdated(), cdr);
      cdr.write_type_2(data.getRegionId());

      cdr.write_type_6(data.getFootYaw());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getRegionOrigin(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRegionOrientation(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRegionNormal(), cdr);
      if(data.getVertexBuffer().size() <= 1000)
      cdr.write_type_e(data.getVertexBuffer());else
          throw new RuntimeException("vertex_buffer field exceeds the maximum length");

      cdr.write_type_2(data.getConcaveHullSize());

      perception_msgs.msg.dds.HeightMapMessagePubSubType.write(data.getLocalHeightMap(), cdr);
   }

   public static void read(perception_msgs.msg.dds.SteppableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLastUpdated(), cdr);	
      data.setRegionId(cdr.read_type_2());
      	
      data.setFootYaw(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRegionOrigin(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRegionOrientation(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRegionNormal(), cdr);	
      cdr.read_type_e(data.getVertexBuffer());	
      data.setConcaveHullSize(cdr.read_type_2());
      	
      perception_msgs.msg.dds.HeightMapMessagePubSubType.read(data.getLocalHeightMap(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SteppableRegionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("last_updated", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastUpdated());

      ser.write_type_2("region_id", data.getRegionId());
      ser.write_type_6("foot_yaw", data.getFootYaw());
      ser.write_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());

      ser.write_type_a("region_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRegionOrientation());

      ser.write_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());

      ser.write_type_e("vertex_buffer", data.getVertexBuffer());
      ser.write_type_2("concave_hull_size", data.getConcaveHullSize());
      ser.write_type_a("local_height_map", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getLocalHeightMap());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SteppableRegionMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("last_updated", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastUpdated());

      data.setRegionId(ser.read_type_2("region_id"));
      data.setFootYaw(ser.read_type_6("foot_yaw"));
      ser.read_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());

      ser.read_type_a("region_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRegionOrientation());

      ser.read_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());

      ser.read_type_e("vertex_buffer", data.getVertexBuffer());
      data.setConcaveHullSize(ser.read_type_2("concave_hull_size"));
      ser.read_type_a("local_height_map", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getLocalHeightMap());

   }

   public static void staticCopy(perception_msgs.msg.dds.SteppableRegionMessage src, perception_msgs.msg.dds.SteppableRegionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SteppableRegionMessage createData()
   {
      return new perception_msgs.msg.dds.SteppableRegionMessage();
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
   
   public void serialize(perception_msgs.msg.dds.SteppableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SteppableRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SteppableRegionMessage src, perception_msgs.msg.dds.SteppableRegionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SteppableRegionMessagePubSubType newInstance()
   {
      return new SteppableRegionMessagePubSubType();
   }
}
