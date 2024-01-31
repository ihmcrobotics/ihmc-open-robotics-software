package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WallPosePacket" defined in "WallPosePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WallPosePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WallPosePacket_.idl instead.
*
*/
public class WallPosePacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.WallPosePacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::WallPosePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "64e9f02137d6e8b970fc3fd72bd99099eb0fca3de216aec68580c73b5694d36a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.WallPosePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.WallPosePacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.WallPosePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.WallPosePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCenterPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getCenterOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getCuttingRadius());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getCenterPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getCenterOrientation(), cdr);
   }

   public static void read(perception_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCuttingRadius(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getCenterPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getCenterOrientation(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("cutting_radius", data.getCuttingRadius());
      ser.write_type_a("center_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPosition());

      ser.write_type_a("center_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getCenterOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.WallPosePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCuttingRadius(ser.read_type_6("cutting_radius"));
      ser.read_type_a("center_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPosition());

      ser.read_type_a("center_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getCenterOrientation());

   }

   public static void staticCopy(perception_msgs.msg.dds.WallPosePacket src, perception_msgs.msg.dds.WallPosePacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.WallPosePacket createData()
   {
      return new perception_msgs.msg.dds.WallPosePacket();
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
   
   public void serialize(perception_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.WallPosePacket src, perception_msgs.msg.dds.WallPosePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WallPosePacketPubSubType newInstance()
   {
      return new WallPosePacketPubSubType();
   }
}
