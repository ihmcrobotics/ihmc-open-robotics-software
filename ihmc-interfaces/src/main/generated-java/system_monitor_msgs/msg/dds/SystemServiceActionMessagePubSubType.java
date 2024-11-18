package system_monitor_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SystemServiceActionMessage" defined in "SystemServiceActionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SystemServiceActionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SystemServiceActionMessage_.idl instead.
*
*/
public class SystemServiceActionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<system_monitor_msgs.msg.dds.SystemServiceActionMessage>
{
   public static final java.lang.String name = "system_monitor_msgs::msg::dds_::SystemServiceActionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e4e82b58a51538e66eff4d803c9e4e031f6d1626a64a60afdbb4b8d190abc39b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, system_monitor_msgs.msg.dds.SystemServiceActionMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(system_monitor_msgs.msg.dds.SystemServiceActionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getServiceName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSystemdAction().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getServiceName().length() <= 255)
      cdr.write_type_d(data.getServiceName());else
          throw new RuntimeException("service_name field exceeds the maximum length");

      if(data.getSystemdAction().length() <= 255)
      cdr.write_type_d(data.getSystemdAction());else
          throw new RuntimeException("systemd_action field exceeds the maximum length");

   }

   public static void read(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getServiceName());	
      cdr.read_type_d(data.getSystemdAction());	

   }

   @Override
   public final void serialize(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("service_name", data.getServiceName());
      ser.write_type_d("systemd_action", data.getSystemdAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, system_monitor_msgs.msg.dds.SystemServiceActionMessage data)
   {
      ser.read_type_d("service_name", data.getServiceName());
      ser.read_type_d("systemd_action", data.getSystemdAction());
   }

   public static void staticCopy(system_monitor_msgs.msg.dds.SystemServiceActionMessage src, system_monitor_msgs.msg.dds.SystemServiceActionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public system_monitor_msgs.msg.dds.SystemServiceActionMessage createData()
   {
      return new system_monitor_msgs.msg.dds.SystemServiceActionMessage();
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
   
   public void serialize(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(system_monitor_msgs.msg.dds.SystemServiceActionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(system_monitor_msgs.msg.dds.SystemServiceActionMessage src, system_monitor_msgs.msg.dds.SystemServiceActionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SystemServiceActionMessagePubSubType newInstance()
   {
      return new SystemServiceActionMessagePubSubType();
   }
}
