package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightMapStateRequestMessage" defined in "HeightMapStateRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightMapStateRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightMapStateRequestMessage_.idl instead.
*
*/
public class HeightMapStateRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeightMapStateRequestMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeightMapStateRequestMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a78981294f1fda0a0eb404a880918b0979766cb3d832b2e653c9792bb2cbeb08";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeightMapStateRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeightMapStateRequestMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapStateRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapStateRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeightMapStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getRequestPause());

      cdr.write_type_7(data.getRequestResume());

      cdr.write_type_7(data.getRequestClear());

   }

   public static void read(perception_msgs.msg.dds.HeightMapStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRequestPause(cdr.read_type_7());
      	
      data.setRequestResume(cdr.read_type_7());
      	
      data.setRequestClear(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.HeightMapStateRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("request_pause", data.getRequestPause());
      ser.write_type_7("request_resume", data.getRequestResume());
      ser.write_type_7("request_clear", data.getRequestClear());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeightMapStateRequestMessage data)
   {
      data.setRequestPause(ser.read_type_7("request_pause"));
      data.setRequestResume(ser.read_type_7("request_resume"));
      data.setRequestClear(ser.read_type_7("request_clear"));
   }

   public static void staticCopy(perception_msgs.msg.dds.HeightMapStateRequestMessage src, perception_msgs.msg.dds.HeightMapStateRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeightMapStateRequestMessage createData()
   {
      return new perception_msgs.msg.dds.HeightMapStateRequestMessage();
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
   
   public void serialize(perception_msgs.msg.dds.HeightMapStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeightMapStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeightMapStateRequestMessage src, perception_msgs.msg.dds.HeightMapStateRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightMapStateRequestMessagePubSubType newInstance()
   {
      return new HeightMapStateRequestMessagePubSubType();
   }
}
