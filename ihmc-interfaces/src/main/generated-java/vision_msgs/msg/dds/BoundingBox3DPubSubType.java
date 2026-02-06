package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BoundingBox3D" defined in "BoundingBox3D_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BoundingBox3D_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BoundingBox3D_.idl instead.
*
*/
public class BoundingBox3DPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.BoundingBox3D>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::BoundingBox3D_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b58ffb38f408bb80973b9c7a7ef46bb1369d377740eff500364c186ff7f34436";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.BoundingBox3D data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.BoundingBox3D data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.BoundingBox3D data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.BoundingBox3D data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCenter(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getSize(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.BoundingBox3D data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.write(data.getCenter(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getSize(), cdr);
   }

   public static void read(vision_msgs.msg.dds.BoundingBox3D data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.read(data.getCenter(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getSize(), cdr);	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.BoundingBox3D data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("center", new geometry_msgs.msg.dds.PosePubSubType(), data.getCenter());

      ser.write_type_a("size", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getSize());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.BoundingBox3D data)
   {
      ser.read_type_a("center", new geometry_msgs.msg.dds.PosePubSubType(), data.getCenter());

      ser.read_type_a("size", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getSize());

   }

   public static void staticCopy(vision_msgs.msg.dds.BoundingBox3D src, vision_msgs.msg.dds.BoundingBox3D dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.BoundingBox3D createData()
   {
      return new vision_msgs.msg.dds.BoundingBox3D();
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
   
   public void serialize(vision_msgs.msg.dds.BoundingBox3D data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.BoundingBox3D data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.BoundingBox3D src, vision_msgs.msg.dds.BoundingBox3D dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BoundingBox3DPubSubType newInstance()
   {
      return new BoundingBox3DPubSubType();
   }
}
