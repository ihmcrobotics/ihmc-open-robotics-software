package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "DetectedFacesPacket" defined in "DetectedFacesPacket_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from DetectedFacesPacket_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit DetectedFacesPacket_.idl instead.
 */
public class DetectedFacesPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DetectedFacesPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DetectedFacesPacket_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public DetectedFacesPacketPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 100; ++a)
      {
         current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DetectedFacesPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DetectedFacesPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getIds().size(); ++a)
      {
         current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIds().get(a).length() + 1;
      }
      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPositions(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {

      if (data.getIds().size() <= 100)
         cdr.write_type_e(data.getIds());
      else
         throw new RuntimeException("ids field exceeds the maximum length");

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPositions(), cdr);
   }

   public static void read(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.read_type_e(data.getIds());

      geometry_msgs.msg.dds.PointPubSubType.read(data.getPositions(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.DetectedFacesPacket src, controller_msgs.msg.dds.DetectedFacesPacket dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DetectedFacesPacket data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("ids", data.getIds());

      ser.write_type_a("positions", new geometry_msgs.msg.dds.PointPubSubType(), data.getPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DetectedFacesPacket data)
   {
      ser.read_type_e("ids", data.getIds());

      ser.read_type_a("positions", new geometry_msgs.msg.dds.PointPubSubType(), data.getPositions());
   }

   @Override
   public controller_msgs.msg.dds.DetectedFacesPacket createData()
   {
      return new controller_msgs.msg.dds.DetectedFacesPacket();
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

   public void serialize(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.DetectedFacesPacket src, controller_msgs.msg.dds.DetectedFacesPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedFacesPacketPubSubType newInstance()
   {
      return new DetectedFacesPacketPubSubType();
   }
}