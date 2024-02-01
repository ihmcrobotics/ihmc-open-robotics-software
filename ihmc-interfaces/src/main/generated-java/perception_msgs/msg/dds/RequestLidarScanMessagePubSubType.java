package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RequestLidarScanMessage" defined in "RequestLidarScanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RequestLidarScanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RequestLidarScanMessage_.idl instead.
*
*/
public class RequestLidarScanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.RequestLidarScanMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::RequestLidarScanMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b49e21c190b37f1f5a0b983d63d9910398dc472fcec9f0f47a76925affa9962c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.RequestLidarScanMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RequestLidarScanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RequestLidarScanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getRemoveShadows());

      cdr.write_type_7(data.getRemoveSelfCollisions());

   }

   public static void read(perception_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRemoveShadows(cdr.read_type_7());
      	
      data.setRemoveSelfCollisions(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("remove_shadows", data.getRemoveShadows());
      ser.write_type_7("remove_self_collisions", data.getRemoveSelfCollisions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.RequestLidarScanMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRemoveShadows(ser.read_type_7("remove_shadows"));
      data.setRemoveSelfCollisions(ser.read_type_7("remove_self_collisions"));
   }

   public static void staticCopy(perception_msgs.msg.dds.RequestLidarScanMessage src, perception_msgs.msg.dds.RequestLidarScanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.RequestLidarScanMessage createData()
   {
      return new perception_msgs.msg.dds.RequestLidarScanMessage();
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
   
   public void serialize(perception_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.RequestLidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.RequestLidarScanMessage src, perception_msgs.msg.dds.RequestLidarScanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestLidarScanMessagePubSubType newInstance()
   {
      return new RequestLidarScanMessagePubSubType();
   }
}
