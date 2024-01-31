package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "InvalidPacketNotificationPacket" defined in "InvalidPacketNotificationPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from InvalidPacketNotificationPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit InvalidPacketNotificationPacket_.idl instead.
*
*/
public class InvalidPacketNotificationPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.InvalidPacketNotificationPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::InvalidPacketNotificationPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "74270feafcbb553bcfba3d146fcc0d7fcf49190fd4dadbfc3dac44cb3a2967ea";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.InvalidPacketNotificationPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.InvalidPacketNotificationPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getPacketClassSimpleName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getErrorMessage().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getPacketClassSimpleName().length() <= 255)
      cdr.write_type_d(data.getPacketClassSimpleName());else
          throw new RuntimeException("packet_class_simple_name field exceeds the maximum length");

      if(data.getErrorMessage().length() <= 255)
      cdr.write_type_d(data.getErrorMessage());else
          throw new RuntimeException("error_message field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getPacketClassSimpleName());	
      cdr.read_type_d(data.getErrorMessage());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("packet_class_simple_name", data.getPacketClassSimpleName());
      ser.write_type_d("error_message", data.getErrorMessage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.InvalidPacketNotificationPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("packet_class_simple_name", data.getPacketClassSimpleName());
      ser.read_type_d("error_message", data.getErrorMessage());
   }

   public static void staticCopy(controller_msgs.msg.dds.InvalidPacketNotificationPacket src, controller_msgs.msg.dds.InvalidPacketNotificationPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.InvalidPacketNotificationPacket createData()
   {
      return new controller_msgs.msg.dds.InvalidPacketNotificationPacket();
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
   
   public void serialize(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.InvalidPacketNotificationPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.InvalidPacketNotificationPacket src, controller_msgs.msg.dds.InvalidPacketNotificationPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public InvalidPacketNotificationPacketPubSubType newInstance()
   {
      return new InvalidPacketNotificationPacketPubSubType();
   }
}
