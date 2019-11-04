package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DoorParameterPacket" defined in "DoorParameterPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DoorParameterPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DoorParameterPacket_.idl instead.
*
*/
public class DoorParameterPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DoorParameterPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DoorParameterPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DoorParameterPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DoorParameterPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DoorParameterPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DoorParameterPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getHingedPointOnGround(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getEndPointOnGround(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getDoorHandleTransformToWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DoorParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getHingedPointOnGround(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getEndPointOnGround(), cdr);
      cdr.write_type_6(data.getDoorHeight());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getDoorHandleTransformToWorld(), cdr);
   }

   public static void read(controller_msgs.msg.dds.DoorParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getHingedPointOnGround(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getEndPointOnGround(), cdr);	
      data.setDoorHeight(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getDoorHandleTransformToWorld(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DoorParameterPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("hinged_point_on_ground", new geometry_msgs.msg.dds.PointPubSubType(), data.getHingedPointOnGround());

      ser.write_type_a("end_point_on_ground", new geometry_msgs.msg.dds.PointPubSubType(), data.getEndPointOnGround());

      ser.write_type_6("door_height", data.getDoorHeight());
      ser.write_type_a("door_handle_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getDoorHandleTransformToWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DoorParameterPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("hinged_point_on_ground", new geometry_msgs.msg.dds.PointPubSubType(), data.getHingedPointOnGround());

      ser.read_type_a("end_point_on_ground", new geometry_msgs.msg.dds.PointPubSubType(), data.getEndPointOnGround());

      data.setDoorHeight(ser.read_type_6("door_height"));
      ser.read_type_a("door_handle_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getDoorHandleTransformToWorld());

   }

   public static void staticCopy(controller_msgs.msg.dds.DoorParameterPacket src, controller_msgs.msg.dds.DoorParameterPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DoorParameterPacket createData()
   {
      return new controller_msgs.msg.dds.DoorParameterPacket();
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
   
   public void serialize(controller_msgs.msg.dds.DoorParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DoorParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DoorParameterPacket src, controller_msgs.msg.dds.DoorParameterPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DoorParameterPacketPubSubType newInstance()
   {
      return new DoorParameterPacketPubSubType();
   }
}
