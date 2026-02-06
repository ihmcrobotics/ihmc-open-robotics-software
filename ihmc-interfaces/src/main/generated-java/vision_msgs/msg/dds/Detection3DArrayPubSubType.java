package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Detection3DArray" defined in "Detection3DArray_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Detection3DArray_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Detection3DArray_.idl instead.
*
*/
public class Detection3DArrayPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.Detection3DArray>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::Detection3DArray_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ba3b3efabdca43958b41b1c8fab788b1f42c6f4f2d20940e8ddbee17f6e5ddbc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.Detection3DArray data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.Detection3DArray data) throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += vision_msgs.msg.dds.Detection3DPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Detection3DArray data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Detection3DArray data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetections().size(); ++i0)
      {
          current_alignment += vision_msgs.msg.dds.Detection3DPubSubType.getCdrSerializedSize(data.getDetections().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.Detection3DArray data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getDetections().size() <= 100)
      cdr.write_type_e(data.getDetections());else
          throw new RuntimeException("detections field exceeds the maximum length: %d > %d".formatted(data.getDetections().size(), 100));

   }

   public static void read(vision_msgs.msg.dds.Detection3DArray data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getDetections());	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.Detection3DArray data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("detections", data.getDetections());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.Detection3DArray data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("detections", data.getDetections());
   }

   public static void staticCopy(vision_msgs.msg.dds.Detection3DArray src, vision_msgs.msg.dds.Detection3DArray dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.Detection3DArray createData()
   {
      return new vision_msgs.msg.dds.Detection3DArray();
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
   
   public void serialize(vision_msgs.msg.dds.Detection3DArray data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.Detection3DArray data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.Detection3DArray src, vision_msgs.msg.dds.Detection3DArray dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Detection3DArrayPubSubType newInstance()
   {
      return new Detection3DArrayPubSubType();
   }
}
