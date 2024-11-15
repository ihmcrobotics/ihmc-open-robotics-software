package system_monitor_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SystemAvailableMessage" defined in "SystemAvailableMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SystemAvailableMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SystemAvailableMessage_.idl instead.
*
*/
public class SystemAvailableMessagePubSubType implements us.ihmc.pubsub.TopicDataType<system_monitor_msgs.msg.dds.SystemAvailableMessage>
{
   public static final java.lang.String name = "system_monitor_msgs::msg::dds_::SystemAvailableMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f94ad864134dbf577aed7b204fe423fe1379ed591ca7a4b72bf8c0180737699a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(system_monitor_msgs.msg.dds.SystemAvailableMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, system_monitor_msgs.msg.dds.SystemAvailableMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(system_monitor_msgs.msg.dds.SystemAvailableMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(system_monitor_msgs.msg.dds.SystemAvailableMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getHostname().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getInstanceId().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(system_monitor_msgs.msg.dds.SystemAvailableMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getHostname().length() <= 255)
      cdr.write_type_d(data.getHostname());else
          throw new RuntimeException("hostname field exceeds the maximum length");

      if(data.getInstanceId().length() <= 255)
      cdr.write_type_d(data.getInstanceId());else
          throw new RuntimeException("instance_id field exceeds the maximum length");

   }

   public static void read(system_monitor_msgs.msg.dds.SystemAvailableMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getHostname());	
      cdr.read_type_d(data.getInstanceId());	

   }

   @Override
   public final void serialize(system_monitor_msgs.msg.dds.SystemAvailableMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("hostname", data.getHostname());
      ser.write_type_d("instance_id", data.getInstanceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, system_monitor_msgs.msg.dds.SystemAvailableMessage data)
   {
      ser.read_type_d("hostname", data.getHostname());
      ser.read_type_d("instance_id", data.getInstanceId());
   }

   public static void staticCopy(system_monitor_msgs.msg.dds.SystemAvailableMessage src, system_monitor_msgs.msg.dds.SystemAvailableMessage dest)
   {
      dest.set(src);
   }

   @Override
   public system_monitor_msgs.msg.dds.SystemAvailableMessage createData()
   {
      return new system_monitor_msgs.msg.dds.SystemAvailableMessage();
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
   
   public void serialize(system_monitor_msgs.msg.dds.SystemAvailableMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(system_monitor_msgs.msg.dds.SystemAvailableMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(system_monitor_msgs.msg.dds.SystemAvailableMessage src, system_monitor_msgs.msg.dds.SystemAvailableMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SystemAvailableMessagePubSubType newInstance()
   {
      return new SystemAvailableMessagePubSubType();
   }
}
