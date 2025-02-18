package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EnableHPUCommandMessage" defined in "EnableHPUCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EnableHPUCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EnableHPUCommandMessage_.idl instead.
*
*/
public class EnableHPUCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.EnableHPUCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::EnableHPUCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "828e7a7712529fe528366f8696e9deaf1a2ff647f956693e65d9bae84eeabe87";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.EnableHPUCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.EnableHPUCommandMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EnableHPUCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EnableHPUCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.EnableHPUCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEnableHpu());

   }

   public static void read(controller_msgs.msg.dds.EnableHPUCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEnableHpu(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.EnableHPUCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable_hpu", data.getEnableHpu());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.EnableHPUCommandMessage data)
   {
      data.setEnableHpu(ser.read_type_7("enable_hpu"));   }

   public static void staticCopy(controller_msgs.msg.dds.EnableHPUCommandMessage src, controller_msgs.msg.dds.EnableHPUCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.EnableHPUCommandMessage createData()
   {
      return new controller_msgs.msg.dds.EnableHPUCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.EnableHPUCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.EnableHPUCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.EnableHPUCommandMessage src, controller_msgs.msg.dds.EnableHPUCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EnableHPUCommandMessagePubSubType newInstance()
   {
      return new EnableHPUCommandMessagePubSubType();
   }
}
