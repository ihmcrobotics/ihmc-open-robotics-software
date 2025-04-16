package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "UwbSwitch" defined in "UwbSwitch_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from UwbSwitch_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit UwbSwitch_.idl instead.
*
*/
public class UwbSwitchPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.UwbSwitch>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::UwbSwitch_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c964d3002b54a0fadda31911ad1bf9ec458e82b1e95a6bcb51d1809a73842eab";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.UwbSwitch data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.UwbSwitch data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.UwbSwitch data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.UwbSwitch data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.UwbSwitch data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getEnabled());

   }

   public static void read(unitree_go_msgs.msg.dds.UwbSwitch data, us.ihmc.idl.CDR cdr)
   {
      data.setEnabled(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.UwbSwitch data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("enabled", data.getEnabled());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.UwbSwitch data)
   {
      data.setEnabled(ser.read_type_9("enabled"));   }

   public static void staticCopy(unitree_go_msgs.msg.dds.UwbSwitch src, unitree_go_msgs.msg.dds.UwbSwitch dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.UwbSwitch createData()
   {
      return new unitree_go_msgs.msg.dds.UwbSwitch();
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
   
   public void serialize(unitree_go_msgs.msg.dds.UwbSwitch data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.UwbSwitch data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.UwbSwitch src, unitree_go_msgs.msg.dds.UwbSwitch dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public UwbSwitchPubSubType newInstance()
   {
      return new UwbSwitchPubSubType();
   }
}
