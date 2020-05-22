package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VehiclePosePacket" defined in "VehiclePosePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VehiclePosePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VehiclePosePacket_.idl instead.
*
*/
public class VehiclePosePacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.VehiclePosePacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::VehiclePosePacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.VehiclePosePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.VehiclePosePacket data) throws java.io.IOException
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


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VehiclePosePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VehiclePosePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.VehiclePosePacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);

      cdr.write_type_2(data.getIndex());

   }

   public static void read(controller_msgs.msg.dds.VehiclePosePacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	

      data.setIndex(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.VehiclePosePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());


      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.write_type_2("index", data.getIndex());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.VehiclePosePacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());


      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      data.setIndex(ser.read_type_2("index"));
   }

   public static void staticCopy(controller_msgs.msg.dds.VehiclePosePacket src, controller_msgs.msg.dds.VehiclePosePacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.VehiclePosePacket createData()
   {
      return new controller_msgs.msg.dds.VehiclePosePacket();
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
   
   public void serialize(controller_msgs.msg.dds.VehiclePosePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.VehiclePosePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.VehiclePosePacket src, controller_msgs.msg.dds.VehiclePosePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VehiclePosePacketPubSubType newInstance()
   {
      return new VehiclePosePacketPubSubType();
   }
}
