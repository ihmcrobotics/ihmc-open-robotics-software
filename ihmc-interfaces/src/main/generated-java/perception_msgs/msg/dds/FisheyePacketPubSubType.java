package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FisheyePacket" defined in "FisheyePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FisheyePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FisheyePacket_.idl instead.
*
*/
public class FisheyePacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.FisheyePacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::FisheyePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b8b166619c7383eafa06c618caca3445319fb54ddc90cc0e2e44b05bd3bf3eac";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.FisheyePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.FisheyePacket data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.VideoPacketPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FisheyePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FisheyePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.VideoPacketPubSubType.getCdrSerializedSize(data.getVideoPacket(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FisheyePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      perception_msgs.msg.dds.VideoPacketPubSubType.write(data.getVideoPacket(), cdr);
   }

   public static void read(perception_msgs.msg.dds.FisheyePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      perception_msgs.msg.dds.VideoPacketPubSubType.read(data.getVideoPacket(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FisheyePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("video_packet", new perception_msgs.msg.dds.VideoPacketPubSubType(), data.getVideoPacket());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FisheyePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("video_packet", new perception_msgs.msg.dds.VideoPacketPubSubType(), data.getVideoPacket());

   }

   public static void staticCopy(perception_msgs.msg.dds.FisheyePacket src, perception_msgs.msg.dds.FisheyePacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.FisheyePacket createData()
   {
      return new perception_msgs.msg.dds.FisheyePacket();
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
   
   public void serialize(perception_msgs.msg.dds.FisheyePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.FisheyePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.FisheyePacket src, perception_msgs.msg.dds.FisheyePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FisheyePacketPubSubType newInstance()
   {
      return new FisheyePacketPubSubType();
   }
}
