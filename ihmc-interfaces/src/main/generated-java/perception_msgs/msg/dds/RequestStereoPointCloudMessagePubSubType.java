package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RequestStereoPointCloudMessage" defined in "RequestStereoPointCloudMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RequestStereoPointCloudMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RequestStereoPointCloudMessage_.idl instead.
*
*/
public class RequestStereoPointCloudMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.RequestStereoPointCloudMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::RequestStereoPointCloudMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bd89dee037898c230033efbd5b7865731e8d0d2584b984ca4459cd25f76f75e9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.RequestStereoPointCloudMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RequestStereoPointCloudMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

   }

   public static void read(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.RequestStereoPointCloudMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));   }

   public static void staticCopy(perception_msgs.msg.dds.RequestStereoPointCloudMessage src, perception_msgs.msg.dds.RequestStereoPointCloudMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.RequestStereoPointCloudMessage createData()
   {
      return new perception_msgs.msg.dds.RequestStereoPointCloudMessage();
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
   
   public void serialize(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.RequestStereoPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.RequestStereoPointCloudMessage src, perception_msgs.msg.dds.RequestStereoPointCloudMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestStereoPointCloudMessagePubSubType newInstance()
   {
      return new RequestStereoPointCloudMessagePubSubType();
   }
}
