package mission_control_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SystemRebootMessage" defined in "SystemRebootMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SystemRebootMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SystemRebootMessage_.idl instead.
*
*/
public class SystemRebootMessagePubSubType implements us.ihmc.pubsub.TopicDataType<mission_control_msgs.msg.dds.SystemRebootMessage>
{
   public static final java.lang.String name = "mission_control_msgs::msg::dds_::SystemRebootMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(mission_control_msgs.msg.dds.SystemRebootMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, mission_control_msgs.msg.dds.SystemRebootMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(mission_control_msgs.msg.dds.SystemRebootMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(mission_control_msgs.msg.dds.SystemRebootMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(mission_control_msgs.msg.dds.SystemRebootMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getUnusedPlaceholderField());

   }

   public static void read(mission_control_msgs.msg.dds.SystemRebootMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setUnusedPlaceholderField(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(mission_control_msgs.msg.dds.SystemRebootMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("unused_placeholder_field", data.getUnusedPlaceholderField());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, mission_control_msgs.msg.dds.SystemRebootMessage data)
   {
      data.setUnusedPlaceholderField(ser.read_type_7("unused_placeholder_field"));   }

   public static void staticCopy(mission_control_msgs.msg.dds.SystemRebootMessage src, mission_control_msgs.msg.dds.SystemRebootMessage dest)
   {
      dest.set(src);
   }

   @Override
   public mission_control_msgs.msg.dds.SystemRebootMessage createData()
   {
      return new mission_control_msgs.msg.dds.SystemRebootMessage();
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
   
   public void serialize(mission_control_msgs.msg.dds.SystemRebootMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(mission_control_msgs.msg.dds.SystemRebootMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(mission_control_msgs.msg.dds.SystemRebootMessage src, mission_control_msgs.msg.dds.SystemRebootMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SystemRebootMessagePubSubType newInstance()
   {
      return new SystemRebootMessagePubSubType();
   }
}
