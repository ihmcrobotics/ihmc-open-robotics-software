package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CenterposeNodeMessage" defined in "CenterposeNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CenterposeNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CenterposeNodeMessage_.idl instead.
*
*/
public class CenterposeNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.CenterposeNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::CenterposeNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f3ae2d30aab1f4d3393b8e6abc29a3d44e5438e04f9aa50414536294755a5841";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.CenterposeNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.CenterposeNodeMessage data) throws java.io.IOException
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

      for(int i0 = 0; i0 < (8); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      for(int i0 = 0; i0 < (8); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.CenterposeNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.CenterposeNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      for(int i0 = 0; i0 < data.getBoundingBox2dVertices().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBox2dVertices()[i0], current_alignment);
      }
      for(int i0 = 0; i0 < data.getBoundingBoxVertices().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBoxVertices()[i0], current_alignment);
      }
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.CenterposeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      for(int i0 = 0; i0 < data.getBoundingBox2dVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBox2dVertices()[i0], cdr);		
      }

      for(int i0 = 0; i0 < data.getBoundingBoxVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBoxVertices()[i0], cdr);		
      }

      cdr.write_type_7(data.getEnableTracking());

   }

   public static void read(perception_msgs.msg.dds.CenterposeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      for(int i0 = 0; i0 < data.getBoundingBox2dVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBox2dVertices()[i0], cdr);	
      }
      	
      for(int i0 = 0; i0 < data.getBoundingBoxVertices().length; ++i0)
      {
        	geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBoxVertices()[i0], cdr);	
      }
      	
      data.setEnableTracking(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.CenterposeNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_f("bounding_box_2d_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBox2dVertices());
      ser.write_type_f("bounding_box_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxVertices());
      ser.write_type_7("enable_tracking", data.getEnableTracking());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.CenterposeNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.read_type_f("bounding_box_2d_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBox2dVertices());
      ser.read_type_f("bounding_box_vertices", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxVertices());
      data.setEnableTracking(ser.read_type_7("enable_tracking"));
   }

   public static void staticCopy(perception_msgs.msg.dds.CenterposeNodeMessage src, perception_msgs.msg.dds.CenterposeNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.CenterposeNodeMessage createData()
   {
      return new perception_msgs.msg.dds.CenterposeNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.CenterposeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.CenterposeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.CenterposeNodeMessage src, perception_msgs.msg.dds.CenterposeNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CenterposeNodeMessagePubSubType newInstance()
   {
      return new CenterposeNodeMessagePubSubType();
   }
}
