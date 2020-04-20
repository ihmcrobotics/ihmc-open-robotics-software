package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedFacesPacket" defined in "DetectedFacesPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedFacesPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedFacesPacket_.idl instead.
*
*/
public class DetectedFacesPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DetectedFacesPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DetectedFacesPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DetectedFacesPacket data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

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



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getIds().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIds().get(i0).length() + 1;
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPositions().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPositions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      if(data.getIds().size() <= 100)
      cdr.write_type_e(data.getIds());else
          throw new RuntimeException("ids field exceeds the maximum length");


      if(data.getPositions().size() <= 100)
      cdr.write_type_e(data.getPositions());else
          throw new RuntimeException("positions field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      cdr.read_type_e(data.getIds());	

      cdr.read_type_e(data.getPositions());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_e("ids", data.getIds());

      ser.write_type_e("positions", data.getPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DetectedFacesPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_e("ids", data.getIds());

      ser.read_type_e("positions", data.getPositions());
   }

   public static void staticCopy(controller_msgs.msg.dds.DetectedFacesPacket src, controller_msgs.msg.dds.DetectedFacesPacket dest)
   {
      dest.set(src);
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
