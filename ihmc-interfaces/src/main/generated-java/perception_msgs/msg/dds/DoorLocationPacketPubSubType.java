package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DoorLocationPacket" defined in "DoorLocationPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DoorLocationPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DoorLocationPacket_.idl instead.
*
*/
public class DoorLocationPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DoorLocationPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DoorLocationPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ba41e017e756f4afab86b25c7ce435761a0a4700c9d1e3e05471dbd9980e637e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DoorLocationPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DoorLocationPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorLocationPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorLocationPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getDoorTransformToWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorLocationPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getDoorTransformToWorld(), cdr);
      cdr.write_type_9(data.getDetectedDoorType());

      cdr.write_type_7(data.getTrustedPosition());

   }

   public static void read(perception_msgs.msg.dds.DoorLocationPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getDoorTransformToWorld(), cdr);	
      data.setDetectedDoorType(cdr.read_type_9());
      	
      data.setTrustedPosition(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorLocationPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("door_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getDoorTransformToWorld());

      ser.write_type_9("detected_door_type", data.getDetectedDoorType());
      ser.write_type_7("trusted_position", data.getTrustedPosition());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorLocationPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("door_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getDoorTransformToWorld());

      data.setDetectedDoorType(ser.read_type_9("detected_door_type"));
      data.setTrustedPosition(ser.read_type_7("trusted_position"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DoorLocationPacket src, perception_msgs.msg.dds.DoorLocationPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DoorLocationPacket createData()
   {
      return new perception_msgs.msg.dds.DoorLocationPacket();
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
   
   public void serialize(perception_msgs.msg.dds.DoorLocationPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DoorLocationPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DoorLocationPacket src, perception_msgs.msg.dds.DoorLocationPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DoorLocationPacketPubSubType newInstance()
   {
      return new DoorLocationPacketPubSubType();
   }
}
