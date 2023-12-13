package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedFiducialPacket" defined in "DetectedFiducialPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedFiducialPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedFiducialPacket_.idl instead.
*
*/
public class DetectedFiducialPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedFiducialPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedFiducialPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "42779ff520895c59f998e40723bb70a3324807f8d128b9e2399d26eab184eafd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedFiducialPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedFiducialPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedFiducialPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getFiducialTransformToWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBounds().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBounds().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_4(data.getFiducialId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getFiducialTransformToWorld(), cdr);
      if(data.getBounds().size() <= 100)
      cdr.write_type_e(data.getBounds());else
          throw new RuntimeException("bounds field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFiducialId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getFiducialTransformToWorld(), cdr);	
      cdr.read_type_e(data.getBounds());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_4("fiducial_id", data.getFiducialId());
      ser.write_type_a("fiducial_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFiducialTransformToWorld());

      ser.write_type_e("bounds", data.getBounds());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedFiducialPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFiducialId(ser.read_type_4("fiducial_id"));
      ser.read_type_a("fiducial_transform_to_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getFiducialTransformToWorld());

      ser.read_type_e("bounds", data.getBounds());
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedFiducialPacket src, perception_msgs.msg.dds.DetectedFiducialPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedFiducialPacket createData()
   {
      return new perception_msgs.msg.dds.DetectedFiducialPacket();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedFiducialPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedFiducialPacket src, perception_msgs.msg.dds.DetectedFiducialPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedFiducialPacketPubSubType newInstance()
   {
      return new DetectedFiducialPacketPubSubType();
   }
}
