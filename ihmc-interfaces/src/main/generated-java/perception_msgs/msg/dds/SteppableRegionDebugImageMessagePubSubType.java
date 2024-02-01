package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SteppableRegionDebugImageMessage" defined in "SteppableRegionDebugImageMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SteppableRegionDebugImageMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SteppableRegionDebugImageMessage_.idl instead.
*
*/
public class SteppableRegionDebugImageMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SteppableRegionDebugImageMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SteppableRegionDebugImageMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e2c689eda1f81ec5ef16a6902b568e5cfa2439c8bb5c5698e9bc27a81e3f442c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SteppableRegionDebugImageMessage data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (250000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      if(data.getData().size() <= 250000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

      cdr.write_type_3(data.getFormat());

   }

   public static void read(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      cdr.read_type_e(data.getData());	
      data.setFormat(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_e("data", data.getData());
      ser.write_type_3("format", data.getFormat());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SteppableRegionDebugImageMessage data)
   {
      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      ser.read_type_e("data", data.getData());
      data.setFormat(ser.read_type_3("format"));
   }

   public static void staticCopy(perception_msgs.msg.dds.SteppableRegionDebugImageMessage src, perception_msgs.msg.dds.SteppableRegionDebugImageMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SteppableRegionDebugImageMessage createData()
   {
      return new perception_msgs.msg.dds.SteppableRegionDebugImageMessage();
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
   
   public void serialize(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SteppableRegionDebugImageMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SteppableRegionDebugImageMessage src, perception_msgs.msg.dds.SteppableRegionDebugImageMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SteppableRegionDebugImageMessagePubSubType newInstance()
   {
      return new SteppableRegionDebugImageMessagePubSubType();
   }
}
