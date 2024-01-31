package atlas_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AtlasElectricMotorEnablePacket" defined in "AtlasElectricMotorEnablePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AtlasElectricMotorEnablePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AtlasElectricMotorEnablePacket_.idl instead.
*
*/
public class AtlasElectricMotorEnablePacketPubSubType implements us.ihmc.pubsub.TopicDataType<atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket>
{
   public static final java.lang.String name = "atlas_msgs::msg::dds_::AtlasElectricMotorEnablePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "484307c2f6f4576ee0dddc567e694b03030dd7a88b2c9aa5c65b6b9c0d095ad6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getAtlasElectricMotorPacketEnumEnable());

      cdr.write_type_7(data.getEnable());

   }

   public static void read(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setAtlasElectricMotorPacketEnumEnable(cdr.read_type_9());
      	
      data.setEnable(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("atlas_electric_motor_packet_enum_enable", data.getAtlasElectricMotorPacketEnumEnable());
      ser.write_type_7("enable", data.getEnable());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setAtlasElectricMotorPacketEnumEnable(ser.read_type_9("atlas_electric_motor_packet_enum_enable"));
      data.setEnable(ser.read_type_7("enable"));
   }

   public static void staticCopy(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket src, atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket dest)
   {
      dest.set(src);
   }

   @Override
   public atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket createData()
   {
      return new atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket();
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
   
   public void serialize(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket src, atlas_msgs.msg.dds.AtlasElectricMotorEnablePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AtlasElectricMotorEnablePacketPubSubType newInstance()
   {
      return new AtlasElectricMotorEnablePacketPubSubType();
   }
}
