package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepConstraintMessage" defined in "StepConstraintMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepConstraintMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepConstraintMessage_.idl instead.
*
*/
public class StepConstraintMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepConstraintMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepConstraintMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ce7e8e4e4e8cf37f0c8149841f50bfdc543e74c25545c5f382204148f5f2010b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepConstraintMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepConstraintMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepConstraintMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepConstraintMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

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
      current_alignment += (data.getHolePolygonsSize().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepConstraintMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.write(data.getRegionOrigin(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRegionOrientation(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRegionNormal(), cdr);
      if(data.getVertexBuffer().size() <= 1000)
      cdr.write_type_e(data.getVertexBuffer());else
          throw new RuntimeException("vertex_buffer field exceeds the maximum length");

      cdr.write_type_2(data.getConcaveHullSize());

      cdr.write_type_2(data.getNumberOfHolesInRegion());

      if(data.getHolePolygonsSize().size() <= 20)
      cdr.write_type_e(data.getHolePolygonsSize());else
          throw new RuntimeException("hole_polygons_size field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.StepConstraintMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRegionOrigin(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRegionOrientation(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRegionNormal(), cdr);	
      cdr.read_type_e(data.getVertexBuffer());	
      data.setConcaveHullSize(cdr.read_type_2());
      	
      data.setNumberOfHolesInRegion(cdr.read_type_2());
      	
      cdr.read_type_e(data.getHolePolygonsSize());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepConstraintMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());

      ser.write_type_a("region_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRegionOrientation());

      ser.write_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());

      ser.write_type_e("vertex_buffer", data.getVertexBuffer());
      ser.write_type_2("concave_hull_size", data.getConcaveHullSize());
      ser.write_type_2("number_of_holes_in_region", data.getNumberOfHolesInRegion());
      ser.write_type_e("hole_polygons_size", data.getHolePolygonsSize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepConstraintMessage data)
   {
      ser.read_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());

      ser.read_type_a("region_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRegionOrientation());

      ser.read_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());

      ser.read_type_e("vertex_buffer", data.getVertexBuffer());
      data.setConcaveHullSize(ser.read_type_2("concave_hull_size"));
      data.setNumberOfHolesInRegion(ser.read_type_2("number_of_holes_in_region"));
      ser.read_type_e("hole_polygons_size", data.getHolePolygonsSize());
   }

   public static void staticCopy(controller_msgs.msg.dds.StepConstraintMessage src, controller_msgs.msg.dds.StepConstraintMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepConstraintMessage createData()
   {
      return new controller_msgs.msg.dds.StepConstraintMessage();
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
   
   public void serialize(controller_msgs.msg.dds.StepConstraintMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepConstraintMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepConstraintMessage src, controller_msgs.msg.dds.StepConstraintMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepConstraintMessagePubSubType newInstance()
   {
      return new StepConstraintMessagePubSubType();
   }
}
