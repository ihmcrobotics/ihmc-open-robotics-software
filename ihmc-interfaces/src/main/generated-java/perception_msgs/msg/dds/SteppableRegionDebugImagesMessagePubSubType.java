package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SteppableRegionDebugImagesMessage" defined in "SteppableRegionDebugImagesMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SteppableRegionDebugImagesMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SteppableRegionDebugImagesMessage_.idl instead.
*
*/
public class SteppableRegionDebugImagesMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SteppableRegionDebugImagesMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SteppableRegionDebugImagesMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "39fbf2f7fb6f59c8f1dd76d6296682f2fa03fd6e0fe972b90857454601667ca5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.SteppableRegionDebugImageMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.SteppableRegionDebugImageMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSteppabilityImages().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.SteppableRegionDebugImageMessagePubSubType.getCdrSerializedSize(data.getSteppabilityImages().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRegionImages().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.SteppableRegionDebugImageMessagePubSubType.getCdrSerializedSize(data.getRegionImages().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getSteppabilityImages().size() <= 10)
      cdr.write_type_e(data.getSteppabilityImages());else
          throw new RuntimeException("steppability_images field exceeds the maximum length");

      if(data.getRegionImages().size() <= 10)
      cdr.write_type_e(data.getRegionImages());else
          throw new RuntimeException("region_images field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getSteppabilityImages());	
      cdr.read_type_e(data.getRegionImages());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("steppability_images", data.getSteppabilityImages());
      ser.write_type_e("region_images", data.getRegionImages());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data)
   {
      ser.read_type_e("steppability_images", data.getSteppabilityImages());
      ser.read_type_e("region_images", data.getRegionImages());
   }

   public static void staticCopy(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage src, perception_msgs.msg.dds.SteppableRegionDebugImagesMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SteppableRegionDebugImagesMessage createData()
   {
      return new perception_msgs.msg.dds.SteppableRegionDebugImagesMessage();
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
   
   public void serialize(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SteppableRegionDebugImagesMessage src, perception_msgs.msg.dds.SteppableRegionDebugImagesMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SteppableRegionDebugImagesMessagePubSubType newInstance()
   {
      return new SteppableRegionDebugImagesMessagePubSubType();
   }
}
