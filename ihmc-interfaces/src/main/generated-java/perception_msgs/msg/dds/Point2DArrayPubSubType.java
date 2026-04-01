package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Point2DArray" defined in "Point2DArray_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Point2DArray_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Point2DArray_.idl instead.
*
*/
public class Point2DArrayPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.Point2DArray>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::Point2DArray_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d718a13e6762f71ba8b5cc62adc32911d78f801a00a9411d5dc9d242cb52ff20";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.Point2DArray data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.Point2DArray data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += vision_msgs.msg.dds.Point2DPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.Point2DArray data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.Point2DArray data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPoints().size(); ++i0)
      {
          current_alignment += vision_msgs.msg.dds.Point2DPubSubType.getCdrSerializedSize(data.getPoints().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.Point2DArray data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPoints().size() <= 100)
      cdr.write_type_e(data.getPoints());else
          throw new RuntimeException("points field exceeds the maximum length: %d > %d".formatted(data.getPoints().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.Point2DArray data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPoints());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.Point2DArray data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("points", data.getPoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.Point2DArray data)
   {
      ser.read_type_e("points", data.getPoints());
   }

   public static void staticCopy(perception_msgs.msg.dds.Point2DArray src, perception_msgs.msg.dds.Point2DArray dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.Point2DArray createData()
   {
      return new perception_msgs.msg.dds.Point2DArray();
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
   
   public void serialize(perception_msgs.msg.dds.Point2DArray data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.Point2DArray data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.Point2DArray src, perception_msgs.msg.dds.Point2DArray dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Point2DArrayPubSubType newInstance()
   {
      return new Point2DArrayPubSubType();
   }
}
