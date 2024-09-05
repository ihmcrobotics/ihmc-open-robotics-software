package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SRTStreamRequest" defined in "SRTStreamRequest_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SRTStreamRequest_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SRTStreamRequest_.idl instead.
*
*/
public class SRTStreamRequestPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SRTStreamRequest>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SRTStreamRequest_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "80a44158d0f237614b762209e265c14b1936ab7fdb1f4d9790c02e9cb39161e7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SRTStreamRequest data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SRTStreamRequest data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SRTStreamRequest data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SRTStreamRequest data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getTopicName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getReceiverAddress().length() + 1;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SRTStreamRequest data, us.ihmc.idl.CDR cdr)
   {
      if(data.getTopicName().length() <= 255)
      cdr.write_type_d(data.getTopicName());else
          throw new RuntimeException("topic_name field exceeds the maximum length");

      if(data.getReceiverAddress().length() <= 255)
      cdr.write_type_d(data.getReceiverAddress());else
          throw new RuntimeException("receiver_address field exceeds the maximum length");

      cdr.write_type_3(data.getReceiverPort());

      cdr.write_type_7(data.getConnectionWanted());

   }

   public static void read(perception_msgs.msg.dds.SRTStreamRequest data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getTopicName());	
      cdr.read_type_d(data.getReceiverAddress());	
      data.setReceiverPort(cdr.read_type_3());
      	
      data.setConnectionWanted(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SRTStreamRequest data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("topic_name", data.getTopicName());
      ser.write_type_d("receiver_address", data.getReceiverAddress());
      ser.write_type_3("receiver_port", data.getReceiverPort());
      ser.write_type_7("connection_wanted", data.getConnectionWanted());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SRTStreamRequest data)
   {
      ser.read_type_d("topic_name", data.getTopicName());
      ser.read_type_d("receiver_address", data.getReceiverAddress());
      data.setReceiverPort(ser.read_type_3("receiver_port"));
      data.setConnectionWanted(ser.read_type_7("connection_wanted"));
   }

   public static void staticCopy(perception_msgs.msg.dds.SRTStreamRequest src, perception_msgs.msg.dds.SRTStreamRequest dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SRTStreamRequest createData()
   {
      return new perception_msgs.msg.dds.SRTStreamRequest();
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
   
   public void serialize(perception_msgs.msg.dds.SRTStreamRequest data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SRTStreamRequest data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SRTStreamRequest src, perception_msgs.msg.dds.SRTStreamRequest dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SRTStreamRequestPubSubType newInstance()
   {
      return new SRTStreamRequestPubSubType();
   }
}
