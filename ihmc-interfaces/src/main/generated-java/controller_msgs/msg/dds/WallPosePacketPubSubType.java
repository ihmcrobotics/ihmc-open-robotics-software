package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WallPosePacket" defined in "WallPosePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WallPosePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WallPosePacket_.idl instead.
*
*/
public class WallPosePacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WallPosePacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WallPosePacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WallPosePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WallPosePacket data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WallPosePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WallPosePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCenterPosition(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getCenterOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_6(data.getCuttingRadius());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getCenterPosition(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getCenterOrientation(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setCuttingRadius(cdr.read_type_6());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getCenterPosition(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getCenterOrientation(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_6("cutting_radius", data.getCuttingRadius());

      ser.write_type_a("center_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPosition());


      ser.write_type_a("center_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getCenterOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WallPosePacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setCuttingRadius(ser.read_type_6("cutting_radius"));

      ser.read_type_a("center_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterPosition());


      ser.read_type_a("center_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getCenterOrientation());

   }

   public static void staticCopy(controller_msgs.msg.dds.WallPosePacket src, controller_msgs.msg.dds.WallPosePacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WallPosePacket createData()
   {
      return new controller_msgs.msg.dds.WallPosePacket();
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
   
   public void serialize(controller_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WallPosePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WallPosePacket src, controller_msgs.msg.dds.WallPosePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WallPosePacketPubSubType newInstance()
   {
      return new WallPosePacketPubSubType();
   }
}
