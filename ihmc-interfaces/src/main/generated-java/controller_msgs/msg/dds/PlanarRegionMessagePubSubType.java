package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PlanarRegionMessage" defined in "PlanarRegionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PlanarRegionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PlanarRegionMessage_.idl instead.
*
*/
public class PlanarRegionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PlanarRegionMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PlanarRegionMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PlanarRegionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PlanarRegionMessage data) throws java.io.IOException
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


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarRegionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarRegionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRegionOrigin(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRegionOrientation(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRegionNormal(), current_alignment);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getVertexBuffer().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getVertexBuffer().get(i0), current_alignment);}


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getConvexPolygonsSize().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PlanarRegionMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_2(data.getRegionId());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getRegionOrigin(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRegionOrientation(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRegionNormal(), cdr);

      if(data.getVertexBuffer().size() <= 1000)
      cdr.write_type_e(data.getVertexBuffer());else
          throw new RuntimeException("vertex_buffer field exceeds the maximum length");


      cdr.write_type_2(data.getConcaveHullSize());


      cdr.write_type_2(data.getNumberOfConvexPolygons());


      if(data.getConvexPolygonsSize().size() <= 20)
      cdr.write_type_e(data.getConvexPolygonsSize());else
          throw new RuntimeException("convex_polygons_size field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.PlanarRegionMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setRegionId(cdr.read_type_2());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getRegionOrigin(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRegionOrientation(), cdr);	

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRegionNormal(), cdr);	

      cdr.read_type_e(data.getVertexBuffer());	

      data.setConcaveHullSize(cdr.read_type_2());
      	

      data.setNumberOfConvexPolygons(cdr.read_type_2());
      	

      cdr.read_type_e(data.getConvexPolygonsSize());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PlanarRegionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_2("region_id", data.getRegionId());

      ser.write_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());


      ser.write_type_a("region_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRegionOrientation());


      ser.write_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());


      ser.write_type_e("vertex_buffer", data.getVertexBuffer());

      ser.write_type_2("concave_hull_size", data.getConcaveHullSize());

      ser.write_type_2("number_of_convex_polygons", data.getNumberOfConvexPolygons());

      ser.write_type_e("convex_polygons_size", data.getConvexPolygonsSize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PlanarRegionMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setRegionId(ser.read_type_2("region_id"));

      ser.read_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());


      ser.read_type_a("region_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRegionOrientation());


      ser.read_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());


      ser.read_type_e("vertex_buffer", data.getVertexBuffer());

      data.setConcaveHullSize(ser.read_type_2("concave_hull_size"));

      data.setNumberOfConvexPolygons(ser.read_type_2("number_of_convex_polygons"));

      ser.read_type_e("convex_polygons_size", data.getConvexPolygonsSize());
   }

   public static void staticCopy(controller_msgs.msg.dds.PlanarRegionMessage src, controller_msgs.msg.dds.PlanarRegionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PlanarRegionMessage createData()
   {
      return new controller_msgs.msg.dds.PlanarRegionMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PlanarRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PlanarRegionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PlanarRegionMessage src, controller_msgs.msg.dds.PlanarRegionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PlanarRegionMessagePubSubType newInstance()
   {
      return new PlanarRegionMessagePubSubType();
   }
}
