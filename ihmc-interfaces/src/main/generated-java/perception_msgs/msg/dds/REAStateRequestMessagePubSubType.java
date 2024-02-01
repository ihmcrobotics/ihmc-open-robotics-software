package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "REAStateRequestMessage" defined in "REAStateRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from REAStateRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit REAStateRequestMessage_.idl instead.
*
*/
public class REAStateRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.REAStateRequestMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::REAStateRequestMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "53711703981f43a001b1547a25a67d7d0a9598fd6e371bdbde0e7b9ce7dbd12f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.REAStateRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.REAStateRequestMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.REAStateRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.REAStateRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.REAStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getRequestPause());

      cdr.write_type_7(data.getRequestResume());

      cdr.write_type_7(data.getRequestClear());

   }

   public static void read(perception_msgs.msg.dds.REAStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRequestPause(cdr.read_type_7());
      	
      data.setRequestResume(cdr.read_type_7());
      	
      data.setRequestClear(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.REAStateRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("request_pause", data.getRequestPause());
      ser.write_type_7("request_resume", data.getRequestResume());
      ser.write_type_7("request_clear", data.getRequestClear());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.REAStateRequestMessage data)
   {
      data.setRequestPause(ser.read_type_7("request_pause"));
      data.setRequestResume(ser.read_type_7("request_resume"));
      data.setRequestClear(ser.read_type_7("request_clear"));
   }

   public static void staticCopy(perception_msgs.msg.dds.REAStateRequestMessage src, perception_msgs.msg.dds.REAStateRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.REAStateRequestMessage createData()
   {
      return new perception_msgs.msg.dds.REAStateRequestMessage();
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
   
   public void serialize(perception_msgs.msg.dds.REAStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.REAStateRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.REAStateRequestMessage src, perception_msgs.msg.dds.REAStateRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public REAStateRequestMessagePubSubType newInstance()
   {
      return new REAStateRequestMessagePubSubType();
   }
}
