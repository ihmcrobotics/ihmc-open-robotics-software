package atlas_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AtlasWristSensorCalibrationRequestPacket" defined in "AtlasWristSensorCalibrationRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AtlasWristSensorCalibrationRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AtlasWristSensorCalibrationRequestPacket_.idl instead.
*
*/
public class AtlasWristSensorCalibrationRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket>
{
   public static final java.lang.String name = "atlas_msgs::msg::dds_::AtlasWristSensorCalibrationRequestPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d5d1f7cce3757fc425c51ff5bf407af4ce6b4345053ab61815f9df33f0cdb2ad";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

   }

   public static void read(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
   }

   public static void staticCopy(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket src, atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket createData()
   {
      return new atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket();
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
   
   public void serialize(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket src, atlas_msgs.msg.dds.AtlasWristSensorCalibrationRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AtlasWristSensorCalibrationRequestPacketPubSubType newInstance()
   {
      return new AtlasWristSensorCalibrationRequestPacketPubSubType();
   }
}
