package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Point2D" defined in "Point2D_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Point2D_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Point2D_.idl instead.
*
*/
public class Point2DPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.Point2D>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::Point2D_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "88a577c3dc1de772665fc69ce4afcfbf68a17f2e896784cd271828e8db1be732";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.Point2D data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.Point2D data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Point2D data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Point2D data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.Point2D data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getX());

      cdr.write_type_6(data.getY());

   }

   public static void read(vision_msgs.msg.dds.Point2D data, us.ihmc.idl.CDR cdr)
   {
      data.setX(cdr.read_type_6());
      	
      data.setY(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.Point2D data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("x", data.getX());
      ser.write_type_6("y", data.getY());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.Point2D data)
   {
      data.setX(ser.read_type_6("x"));
      data.setY(ser.read_type_6("y"));
   }

   public static void staticCopy(vision_msgs.msg.dds.Point2D src, vision_msgs.msg.dds.Point2D dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.Point2D createData()
   {
      return new vision_msgs.msg.dds.Point2D();
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
   
   public void serialize(vision_msgs.msg.dds.Point2D data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.Point2D data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.Point2D src, vision_msgs.msg.dds.Point2D dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Point2DPubSubType newInstance()
   {
      return new Point2DPubSubType();
   }
}
