package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorControlModePacket" defined in "BehaviorControlModePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorControlModePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorControlModePacket_.idl instead.
*
*/
public class BehaviorControlModePacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.BehaviorControlModePacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::BehaviorControlModePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3d151d9c43914225f980958658335ac43237bbba952b2f09965f68409f8e855e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.BehaviorControlModePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.BehaviorControlModePacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.BehaviorControlModePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.BehaviorControlModePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.BehaviorControlModePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getBehaviorControlModeEnumRequest());

   }

   public static void read(toolbox_msgs.msg.dds.BehaviorControlModePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setBehaviorControlModeEnumRequest(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.BehaviorControlModePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("behavior_control_mode_enum_request", data.getBehaviorControlModeEnumRequest());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.BehaviorControlModePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setBehaviorControlModeEnumRequest(ser.read_type_9("behavior_control_mode_enum_request"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.BehaviorControlModePacket src, toolbox_msgs.msg.dds.BehaviorControlModePacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.BehaviorControlModePacket createData()
   {
      return new toolbox_msgs.msg.dds.BehaviorControlModePacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.BehaviorControlModePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.BehaviorControlModePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.BehaviorControlModePacket src, toolbox_msgs.msg.dds.BehaviorControlModePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorControlModePacketPubSubType newInstance()
   {
      return new BehaviorControlModePacketPubSubType();
   }
}
