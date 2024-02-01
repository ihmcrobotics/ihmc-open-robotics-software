package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SteppableRegionsListCollectionMessage" defined in "SteppableRegionsListCollectionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SteppableRegionsListCollectionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SteppableRegionsListCollectionMessage_.idl instead.
*
*/
public class SteppableRegionsListCollectionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SteppableRegionsListCollectionMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SteppableRegionsListCollectionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "da9c10b8b757c40787dfee75ca8b92675b7507156e0007ec60716fffcb2d2411";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 3000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLastUpdated(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFootYaw().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRegionsPerYaw().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRegionId().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRegionOrigin().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRegionOrigin().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRegionOrientation().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRegionOrientation().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRegionNormal().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRegionNormal().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getConcaveHullsSize().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getVertexBuffer().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getVertexBuffer().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLocalHeightMap().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getLocalHeightMap().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLastUpdated(), cdr);
      if(data.getFootYaw().size() <= 100)
      cdr.write_type_e(data.getFootYaw());else
          throw new RuntimeException("foot_yaw field exceeds the maximum length");

      if(data.getRegionsPerYaw().size() <= 100)
      cdr.write_type_e(data.getRegionsPerYaw());else
          throw new RuntimeException("regions_per_yaw field exceeds the maximum length");

      if(data.getRegionId().size() <= 1000)
      cdr.write_type_e(data.getRegionId());else
          throw new RuntimeException("region_id field exceeds the maximum length");

      if(data.getRegionOrigin().size() <= 1000)
      cdr.write_type_e(data.getRegionOrigin());else
          throw new RuntimeException("region_origin field exceeds the maximum length");

      if(data.getRegionOrientation().size() <= 1000)
      cdr.write_type_e(data.getRegionOrientation());else
          throw new RuntimeException("region_orientation field exceeds the maximum length");

      if(data.getRegionNormal().size() <= 1000)
      cdr.write_type_e(data.getRegionNormal());else
          throw new RuntimeException("region_normal field exceeds the maximum length");

      if(data.getConcaveHullsSize().size() <= 100)
      cdr.write_type_e(data.getConcaveHullsSize());else
          throw new RuntimeException("concave_hulls_size field exceeds the maximum length");

      if(data.getVertexBuffer().size() <= 3000)
      cdr.write_type_e(data.getVertexBuffer());else
          throw new RuntimeException("vertex_buffer field exceeds the maximum length");

      if(data.getLocalHeightMap().size() <= 100)
      cdr.write_type_e(data.getLocalHeightMap());else
          throw new RuntimeException("local_height_map field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLastUpdated(), cdr);	
      cdr.read_type_e(data.getFootYaw());	
      cdr.read_type_e(data.getRegionsPerYaw());	
      cdr.read_type_e(data.getRegionId());	
      cdr.read_type_e(data.getRegionOrigin());	
      cdr.read_type_e(data.getRegionOrientation());	
      cdr.read_type_e(data.getRegionNormal());	
      cdr.read_type_e(data.getConcaveHullsSize());	
      cdr.read_type_e(data.getVertexBuffer());	
      cdr.read_type_e(data.getLocalHeightMap());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("last_updated", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastUpdated());

      ser.write_type_e("foot_yaw", data.getFootYaw());
      ser.write_type_e("regions_per_yaw", data.getRegionsPerYaw());
      ser.write_type_e("region_id", data.getRegionId());
      ser.write_type_e("region_origin", data.getRegionOrigin());
      ser.write_type_e("region_orientation", data.getRegionOrientation());
      ser.write_type_e("region_normal", data.getRegionNormal());
      ser.write_type_e("concave_hulls_size", data.getConcaveHullsSize());
      ser.write_type_e("vertex_buffer", data.getVertexBuffer());
      ser.write_type_e("local_height_map", data.getLocalHeightMap());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("last_updated", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastUpdated());

      ser.read_type_e("foot_yaw", data.getFootYaw());
      ser.read_type_e("regions_per_yaw", data.getRegionsPerYaw());
      ser.read_type_e("region_id", data.getRegionId());
      ser.read_type_e("region_origin", data.getRegionOrigin());
      ser.read_type_e("region_orientation", data.getRegionOrientation());
      ser.read_type_e("region_normal", data.getRegionNormal());
      ser.read_type_e("concave_hulls_size", data.getConcaveHullsSize());
      ser.read_type_e("vertex_buffer", data.getVertexBuffer());
      ser.read_type_e("local_height_map", data.getLocalHeightMap());
   }

   public static void staticCopy(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage src, perception_msgs.msg.dds.SteppableRegionsListCollectionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SteppableRegionsListCollectionMessage createData()
   {
      return new perception_msgs.msg.dds.SteppableRegionsListCollectionMessage();
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
   
   public void serialize(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SteppableRegionsListCollectionMessage src, perception_msgs.msg.dds.SteppableRegionsListCollectionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SteppableRegionsListCollectionMessagePubSubType newInstance()
   {
      return new SteppableRegionsListCollectionMessagePubSubType();
   }
}
