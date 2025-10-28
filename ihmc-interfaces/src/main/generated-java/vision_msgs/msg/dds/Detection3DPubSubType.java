package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Detection3D" defined in "Detection3D_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Detection3D_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Detection3D_.idl instead.
*
*/
public class Detection3DPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.Detection3D>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::Detection3D_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6ae7f212272e6887c426698fd3f34107ad2c44e14327741aa61e8902e2f91573";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.Detection3D data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.Detection3D data) throws java.io.IOException
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
          current_alignment += vision_msgs.msg.dds.ObjectHypothesisWithPosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += vision_msgs.msg.dds.BoundingBox3DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Detection3D data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Detection3D data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getResults().size(); ++i0)
      {
          current_alignment += vision_msgs.msg.dds.ObjectHypothesisWithPosePubSubType.getCdrSerializedSize(data.getResults().get(i0), current_alignment);}

      current_alignment += vision_msgs.msg.dds.BoundingBox3DPubSubType.getCdrSerializedSize(data.getBbox(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getId().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.Detection3D data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getResults().size() <= 100)
      cdr.write_type_e(data.getResults());else
          throw new RuntimeException("results field exceeds the maximum length: %d > %d".formatted(data.getResults().size(), 100));

      vision_msgs.msg.dds.BoundingBox3DPubSubType.write(data.getBbox(), cdr);
      if(data.getId().length() <= 255)
      cdr.write_type_d(data.getId());else
          throw new RuntimeException("id field exceeds the maximum length: %d > %d".formatted(data.getId().length(), 255));

   }

   public static void read(vision_msgs.msg.dds.Detection3D data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getResults());	
      vision_msgs.msg.dds.BoundingBox3DPubSubType.read(data.getBbox(), cdr);	
      cdr.read_type_d(data.getId());	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.Detection3D data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("results", data.getResults());
      ser.write_type_a("bbox", new vision_msgs.msg.dds.BoundingBox3DPubSubType(), data.getBbox());

      ser.write_type_d("id", data.getId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.Detection3D data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("results", data.getResults());
      ser.read_type_a("bbox", new vision_msgs.msg.dds.BoundingBox3DPubSubType(), data.getBbox());

      ser.read_type_d("id", data.getId());
   }

   public static void staticCopy(vision_msgs.msg.dds.Detection3D src, vision_msgs.msg.dds.Detection3D dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.Detection3D createData()
   {
      return new vision_msgs.msg.dds.Detection3D();
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
   
   public void serialize(vision_msgs.msg.dds.Detection3D data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.Detection3D data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.Detection3D src, vision_msgs.msg.dds.Detection3D dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Detection3DPubSubType newInstance()
   {
      return new Detection3DPubSubType();
   }
}
