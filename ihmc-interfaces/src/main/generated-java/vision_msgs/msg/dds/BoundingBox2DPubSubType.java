package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BoundingBox2D" defined in "BoundingBox2D_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BoundingBox2D_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BoundingBox2D_.idl instead.
*
*/
public class BoundingBox2DPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.BoundingBox2D>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::BoundingBox2D_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fd1044549e1e1784ae150d67c958621a7dfc93e54a4e00ed45f6ab13c00715f2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.BoundingBox2D data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.BoundingBox2D data) throws java.io.IOException
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

      current_alignment += vision_msgs.msg.dds.Pose2DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.BoundingBox2D data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.BoundingBox2D data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += vision_msgs.msg.dds.Pose2DPubSubType.getCdrSerializedSize(data.getCenter(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.BoundingBox2D data, us.ihmc.idl.CDR cdr)
   {
      vision_msgs.msg.dds.Pose2DPubSubType.write(data.getCenter(), cdr);
      cdr.write_type_6(data.getSizeX());

      cdr.write_type_6(data.getSizeY());

   }

   public static void read(vision_msgs.msg.dds.BoundingBox2D data, us.ihmc.idl.CDR cdr)
   {
      vision_msgs.msg.dds.Pose2DPubSubType.read(data.getCenter(), cdr);	
      data.setSizeX(cdr.read_type_6());
      	
      data.setSizeY(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.BoundingBox2D data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("center", new vision_msgs.msg.dds.Pose2DPubSubType(), data.getCenter());

      ser.write_type_6("size_x", data.getSizeX());
      ser.write_type_6("size_y", data.getSizeY());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.BoundingBox2D data)
   {
      ser.read_type_a("center", new vision_msgs.msg.dds.Pose2DPubSubType(), data.getCenter());

      data.setSizeX(ser.read_type_6("size_x"));
      data.setSizeY(ser.read_type_6("size_y"));
   }

   public static void staticCopy(vision_msgs.msg.dds.BoundingBox2D src, vision_msgs.msg.dds.BoundingBox2D dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.BoundingBox2D createData()
   {
      return new vision_msgs.msg.dds.BoundingBox2D();
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
   
   public void serialize(vision_msgs.msg.dds.BoundingBox2D data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.BoundingBox2D data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.BoundingBox2D src, vision_msgs.msg.dds.BoundingBox2D dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BoundingBox2DPubSubType newInstance()
   {
      return new BoundingBox2DPubSubType();
   }
}
