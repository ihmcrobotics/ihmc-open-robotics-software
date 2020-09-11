package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PlanarRegionsListMessage" defined in "PlanarRegionsListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PlanarRegionsListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PlanarRegionsListMessage_.idl instead.
*
*/
public class PlanarRegionsListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PlanarRegionsListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PlanarRegionsListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PlanarRegionsListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (3000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 3000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 3000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 3000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (3000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (3000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (3000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarRegionsListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarRegionsListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


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
      current_alignment += (data.getNumberOfConvexPolygons().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getConvexPolygonsSize().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getVertexBuffer().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getVertexBuffer().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getRegionId().size() <= 3000)
      cdr.write_type_e(data.getRegionId());else
          throw new RuntimeException("region_id field exceeds the maximum length");

      if(data.getRegionOrigin().size() <= 3000)
      cdr.write_type_e(data.getRegionOrigin());else
          throw new RuntimeException("region_origin field exceeds the maximum length");

      if(data.getRegionOrientation().size() <= 3000)
      cdr.write_type_e(data.getRegionOrientation());else
          throw new RuntimeException("region_orientation field exceeds the maximum length");

      if(data.getRegionNormal().size() <= 3000)
      cdr.write_type_e(data.getRegionNormal());else
          throw new RuntimeException("region_normal field exceeds the maximum length");

      if(data.getConcaveHullsSize().size() <= 3000)
      cdr.write_type_e(data.getConcaveHullsSize());else
          throw new RuntimeException("concave_hulls_size field exceeds the maximum length");

      if(data.getNumberOfConvexPolygons().size() <= 3000)
      cdr.write_type_e(data.getNumberOfConvexPolygons());else
          throw new RuntimeException("number_of_convex_polygons field exceeds the maximum length");

      if(data.getConvexPolygonsSize().size() <= 3000)
      cdr.write_type_e(data.getConvexPolygonsSize());else
          throw new RuntimeException("convex_polygons_size field exceeds the maximum length");

      if(data.getVertexBuffer().size() <= 50000)
      cdr.write_type_e(data.getVertexBuffer());else
          throw new RuntimeException("vertex_buffer field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getRegionId());	
      cdr.read_type_e(data.getRegionOrigin());	
      cdr.read_type_e(data.getRegionOrientation());	
      cdr.read_type_e(data.getRegionNormal());	
      cdr.read_type_e(data.getConcaveHullsSize());	
      cdr.read_type_e(data.getNumberOfConvexPolygons());	
      cdr.read_type_e(data.getConvexPolygonsSize());	
      cdr.read_type_e(data.getVertexBuffer());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("region_id", data.getRegionId());
      ser.write_type_e("region_origin", data.getRegionOrigin());
      ser.write_type_e("region_orientation", data.getRegionOrientation());
      ser.write_type_e("region_normal", data.getRegionNormal());
      ser.write_type_e("concave_hulls_size", data.getConcaveHullsSize());
      ser.write_type_e("number_of_convex_polygons", data.getNumberOfConvexPolygons());
      ser.write_type_e("convex_polygons_size", data.getConvexPolygonsSize());
      ser.write_type_e("vertex_buffer", data.getVertexBuffer());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PlanarRegionsListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("region_id", data.getRegionId());
      ser.read_type_e("region_origin", data.getRegionOrigin());
      ser.read_type_e("region_orientation", data.getRegionOrientation());
      ser.read_type_e("region_normal", data.getRegionNormal());
      ser.read_type_e("concave_hulls_size", data.getConcaveHullsSize());
      ser.read_type_e("number_of_convex_polygons", data.getNumberOfConvexPolygons());
      ser.read_type_e("convex_polygons_size", data.getConvexPolygonsSize());
      ser.read_type_e("vertex_buffer", data.getVertexBuffer());
   }

   public static void staticCopy(controller_msgs.msg.dds.PlanarRegionsListMessage src, controller_msgs.msg.dds.PlanarRegionsListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PlanarRegionsListMessage createData()
   {
      return new controller_msgs.msg.dds.PlanarRegionsListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PlanarRegionsListMessage src, controller_msgs.msg.dds.PlanarRegionsListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PlanarRegionsListMessagePubSubType newInstance()
   {
      return new PlanarRegionsListMessagePubSubType();
   }
}
