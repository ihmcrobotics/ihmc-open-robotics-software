package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YawPitchRollMessage" defined in "YawPitchRollMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YawPitchRollMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YawPitchRollMessage_.idl instead.
*
*/
public class YawPitchRollMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.YawPitchRollMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::YawPitchRollMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b8c37af85ea83126c0b1af35c8ad7e6c18d61ff31f8dcfe945e2a7f4a5be2e51";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.YawPitchRollMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.YawPitchRollMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getYaw());

      cdr.write_type_6(data.getPitch());

      cdr.write_type_6(data.getRoll());

   }

   public static void read(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setYaw(cdr.read_type_6());
      	
      data.setPitch(cdr.read_type_6());
      	
      data.setRoll(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("yaw", data.getYaw());
      ser.write_type_6("pitch", data.getPitch());
      ser.write_type_6("roll", data.getRoll());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.YawPitchRollMessage data)
   {
      data.setYaw(ser.read_type_6("yaw"));
      data.setPitch(ser.read_type_6("pitch"));
      data.setRoll(ser.read_type_6("roll"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.YawPitchRollMessage src, ihmc_common_msgs.msg.dds.YawPitchRollMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.YawPitchRollMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.YawPitchRollMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.YawPitchRollMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.YawPitchRollMessage src, ihmc_common_msgs.msg.dds.YawPitchRollMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YawPitchRollMessagePubSubType newInstance()
   {
      return new YawPitchRollMessagePubSubType();
   }
}
