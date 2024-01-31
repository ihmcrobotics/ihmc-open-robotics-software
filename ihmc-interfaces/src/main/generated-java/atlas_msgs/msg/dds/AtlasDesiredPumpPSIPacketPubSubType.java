package atlas_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AtlasDesiredPumpPSIPacket" defined in "AtlasDesiredPumpPSIPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AtlasDesiredPumpPSIPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AtlasDesiredPumpPSIPacket_.idl instead.
*
*/
public class AtlasDesiredPumpPSIPacketPubSubType implements us.ihmc.pubsub.TopicDataType<atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket>
{
   public static final java.lang.String name = "atlas_msgs::msg::dds_::AtlasDesiredPumpPSIPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "741c3effd15815663dfddb994e2bd9c7c91e308000d8ff0c82a23faafb66045f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getDesiredPumpPsi());

   }

   public static void read(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setDesiredPumpPsi(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("desired_pump_psi", data.getDesiredPumpPsi());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setDesiredPumpPsi(ser.read_type_2("desired_pump_psi"));
   }

   public static void staticCopy(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket src, atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket dest)
   {
      dest.set(src);
   }

   @Override
   public atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket createData()
   {
      return new atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket();
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
   
   public void serialize(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket src, atlas_msgs.msg.dds.AtlasDesiredPumpPSIPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AtlasDesiredPumpPSIPacketPubSubType newInstance()
   {
      return new AtlasDesiredPumpPSIPacketPubSubType();
   }
}
