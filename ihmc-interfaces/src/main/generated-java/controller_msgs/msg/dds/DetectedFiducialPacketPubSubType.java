package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedFiducialPacket" defined in "DetectedFiducialPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedFiducialPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedFiducialPacket_.idl instead.
*
*/
public class DetectedFiducialPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DetectedFiducialPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DetectedFiducialPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DetectedFiducialPacket data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DetectedFiducialPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DetectedFiducialPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getFiducialTransformToWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_4(data.getFiducialId());


      geometry_msgs.msg.dds.PosePubSubType.write(data.getFiducialTransformToWorld(), cdr);
   }

   public static void read(controller_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setFiducialId(cdr.read_type_4());
      	

      geometry_msgs.msg.dds.PosePubSubType.read(data.getFiducialTransformToWorld(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_4("fiducial_id", data.getFiducialId());

      ser.write_type_a("fiducial_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFiducialTransformToWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DetectedFiducialPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setFiducialId(ser.read_type_4("fiducial_id"));

      ser.read_type_a("fiducial_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFiducialTransformToWorld());

   }

   public static void staticCopy(controller_msgs.msg.dds.DetectedFiducialPacket src, controller_msgs.msg.dds.DetectedFiducialPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DetectedFiducialPacket createData()
   {
      return new controller_msgs.msg.dds.DetectedFiducialPacket();
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
   
   public void serialize(controller_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DetectedFiducialPacket src, controller_msgs.msg.dds.DetectedFiducialPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedFiducialPacketPubSubType newInstance()
   {
      return new DetectedFiducialPacketPubSubType();
   }
}
