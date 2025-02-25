package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ROS2CompatImage" defined in "ROS2CompatImage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ROS2CompatImage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ROS2CompatImage_.idl instead.
*
*/
public class ROS2CompatImagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ROS2CompatImage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ROS2CompatImage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "33f0aa1b8ace97b8e75b9595ad558267e4cafa4514f98e7d048da38cf4849e18";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ROS2CompatImage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ROS2CompatImage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (5000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ROS2CompatImage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ROS2CompatImage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getEncoding().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ROS2CompatImage data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      cdr.write_type_4(data.getHeight());

      cdr.write_type_4(data.getWidth());

      if(data.getEncoding().length() <= 255)
      cdr.write_type_d(data.getEncoding());else
          throw new RuntimeException("encoding field exceeds the maximum length");

      cdr.write_type_9(data.getIsBigendian());

      cdr.write_type_4(data.getStep());

      if(data.getData().size() <= 5000000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.ROS2CompatImage data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      data.setHeight(cdr.read_type_4());
      	
      data.setWidth(cdr.read_type_4());
      	
      cdr.read_type_d(data.getEncoding());	
      data.setIsBigendian(cdr.read_type_9());
      	
      data.setStep(cdr.read_type_4());
      	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ROS2CompatImage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_4("height", data.getHeight());
      ser.write_type_4("width", data.getWidth());
      ser.write_type_d("encoding", data.getEncoding());
      ser.write_type_9("is_bigendian", data.getIsBigendian());
      ser.write_type_4("step", data.getStep());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ROS2CompatImage data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      data.setHeight(ser.read_type_4("height"));
      data.setWidth(ser.read_type_4("width"));
      ser.read_type_d("encoding", data.getEncoding());
      data.setIsBigendian(ser.read_type_9("is_bigendian"));
      data.setStep(ser.read_type_4("step"));
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(perception_msgs.msg.dds.ROS2CompatImage src, perception_msgs.msg.dds.ROS2CompatImage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ROS2CompatImage createData()
   {
      return new perception_msgs.msg.dds.ROS2CompatImage();
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
   
   public void serialize(perception_msgs.msg.dds.ROS2CompatImage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ROS2CompatImage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ROS2CompatImage src, perception_msgs.msg.dds.ROS2CompatImage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ROS2CompatImagePubSubType newInstance()
   {
      return new ROS2CompatImagePubSubType();
   }
}
