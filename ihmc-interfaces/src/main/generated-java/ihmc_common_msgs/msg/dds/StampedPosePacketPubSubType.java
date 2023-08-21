package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StampedPosePacket" defined in "StampedPosePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StampedPosePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StampedPosePacket_.idl instead.
*
*/
public class StampedPosePacketPubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.StampedPosePacket>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::StampedPosePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "046998c0678764138cf2292a89f7e9db0645e555bef6f56500deb8d5f42c6c59";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.StampedPosePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.StampedPosePacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.TwistPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.StampedPosePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.StampedPosePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TwistPubSubType.getCdrSerializedSize(data.getTwist(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameId().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.StampedPosePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
      geometry_msgs.msg.dds.TwistPubSubType.write(data.getTwist(), cdr);
      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_6(data.getConfidenceFactor());

      if(data.getFrameId().length() <= 255)
      cdr.write_type_d(data.getFrameId());else
          throw new RuntimeException("frame_id field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.StampedPosePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	
      geometry_msgs.msg.dds.TwistPubSubType.read(data.getTwist(), cdr);	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setConfidenceFactor(cdr.read_type_6());
      	
      cdr.read_type_d(data.getFrameId());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.StampedPosePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.write_type_a("twist", new geometry_msgs.msg.dds.TwistPubSubType(), data.getTwist());

      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_6("confidence_factor", data.getConfidenceFactor());
      ser.write_type_d("frame_id", data.getFrameId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.StampedPosePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.read_type_a("twist", new geometry_msgs.msg.dds.TwistPubSubType(), data.getTwist());

      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setConfidenceFactor(ser.read_type_6("confidence_factor"));
      ser.read_type_d("frame_id", data.getFrameId());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.StampedPosePacket src, ihmc_common_msgs.msg.dds.StampedPosePacket dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.StampedPosePacket createData()
   {
      return new ihmc_common_msgs.msg.dds.StampedPosePacket();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.StampedPosePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.StampedPosePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.StampedPosePacket src, ihmc_common_msgs.msg.dds.StampedPosePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StampedPosePacketPubSubType newInstance()
   {
      return new StampedPosePacketPubSubType();
   }
}
