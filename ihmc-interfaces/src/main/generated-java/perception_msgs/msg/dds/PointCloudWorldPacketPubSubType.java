package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PointCloudWorldPacket" defined in "PointCloudWorldPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PointCloudWorldPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PointCloudWorldPacket_.idl instead.
*
*/
public class PointCloudWorldPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PointCloudWorldPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PointCloudWorldPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a850b95e1a21eb88741f024cd8f24d7b073d17d7552df0b7331a2215067bcbda";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PointCloudWorldPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PointCloudWorldPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PointCloudWorldPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PointCloudWorldPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getGroundQuadTreeSupport().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDecayingWorldScan().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PointCloudWorldPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      if(data.getGroundQuadTreeSupport().size() <= 100)
      cdr.write_type_e(data.getGroundQuadTreeSupport());else
          throw new RuntimeException("ground_quad_tree_support field exceeds the maximum length");

      if(data.getDecayingWorldScan().size() <= 100)
      cdr.write_type_e(data.getDecayingWorldScan());else
          throw new RuntimeException("decaying_world_scan field exceeds the maximum length");

      cdr.write_type_5(data.getDefaultGroundHeight());

   }

   public static void read(perception_msgs.msg.dds.PointCloudWorldPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      cdr.read_type_e(data.getGroundQuadTreeSupport());	
      cdr.read_type_e(data.getDecayingWorldScan());	
      data.setDefaultGroundHeight(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PointCloudWorldPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_e("ground_quad_tree_support", data.getGroundQuadTreeSupport());
      ser.write_type_e("decaying_world_scan", data.getDecayingWorldScan());
      ser.write_type_5("default_ground_height", data.getDefaultGroundHeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PointCloudWorldPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      ser.read_type_e("ground_quad_tree_support", data.getGroundQuadTreeSupport());
      ser.read_type_e("decaying_world_scan", data.getDecayingWorldScan());
      data.setDefaultGroundHeight(ser.read_type_5("default_ground_height"));
   }

   public static void staticCopy(perception_msgs.msg.dds.PointCloudWorldPacket src, perception_msgs.msg.dds.PointCloudWorldPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PointCloudWorldPacket createData()
   {
      return new perception_msgs.msg.dds.PointCloudWorldPacket();
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
   
   public void serialize(perception_msgs.msg.dds.PointCloudWorldPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PointCloudWorldPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PointCloudWorldPacket src, perception_msgs.msg.dds.PointCloudWorldPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PointCloudWorldPacketPubSubType newInstance()
   {
      return new PointCloudWorldPacketPubSubType();
   }
}
