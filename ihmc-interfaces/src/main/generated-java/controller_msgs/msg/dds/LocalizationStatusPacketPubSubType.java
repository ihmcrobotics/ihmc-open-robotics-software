package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LocalizationStatusPacket" defined in "LocalizationStatusPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LocalizationStatusPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LocalizationStatusPacket_.idl instead.
*
*/
public class LocalizationStatusPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.LocalizationStatusPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::LocalizationStatusPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "80f477d7c4e9880fdf6ea2fc78d3c1052b3c830816a0ca56ff5c16e938386ff0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.LocalizationStatusPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.LocalizationStatusPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LocalizationStatusPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LocalizationStatusPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStatus().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.LocalizationStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getOverlap());

      if(data.getStatus().length() <= 255)
      cdr.write_type_d(data.getStatus());else
          throw new RuntimeException("status field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.LocalizationStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setOverlap(cdr.read_type_6());
      	
      cdr.read_type_d(data.getStatus());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.LocalizationStatusPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("overlap", data.getOverlap());
      ser.write_type_d("status", data.getStatus());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.LocalizationStatusPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setOverlap(ser.read_type_6("overlap"));
      ser.read_type_d("status", data.getStatus());
   }

   public static void staticCopy(controller_msgs.msg.dds.LocalizationStatusPacket src, controller_msgs.msg.dds.LocalizationStatusPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.LocalizationStatusPacket createData()
   {
      return new controller_msgs.msg.dds.LocalizationStatusPacket();
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
   
   public void serialize(controller_msgs.msg.dds.LocalizationStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.LocalizationStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.LocalizationStatusPacket src, controller_msgs.msg.dds.LocalizationStatusPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LocalizationStatusPacketPubSubType newInstance()
   {
      return new LocalizationStatusPacketPubSubType();
   }
}
