package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorStatusPacket" defined in "BehaviorStatusPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorStatusPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorStatusPacket_.idl instead.
*
*/
public class BehaviorStatusPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.BehaviorStatusPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::BehaviorStatusPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7d83efec22c6b3ca50b9c97e17c5c23e663a515a17e6c2e2fb02ade532777d4c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.BehaviorStatusPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.BehaviorStatusPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.BehaviorStatusPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.BehaviorStatusPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.BehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getCurrentBehaviorStatus());

      cdr.write_type_9(data.getHumanoidBehaviorType());

   }

   public static void read(toolbox_msgs.msg.dds.BehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCurrentBehaviorStatus(cdr.read_type_9());
      	
      data.setHumanoidBehaviorType(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.BehaviorStatusPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("current_behavior_status", data.getCurrentBehaviorStatus());
      ser.write_type_9("humanoid_behavior_type", data.getHumanoidBehaviorType());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.BehaviorStatusPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCurrentBehaviorStatus(ser.read_type_9("current_behavior_status"));
      data.setHumanoidBehaviorType(ser.read_type_9("humanoid_behavior_type"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.BehaviorStatusPacket src, toolbox_msgs.msg.dds.BehaviorStatusPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.BehaviorStatusPacket createData()
   {
      return new toolbox_msgs.msg.dds.BehaviorStatusPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.BehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.BehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.BehaviorStatusPacket src, toolbox_msgs.msg.dds.BehaviorStatusPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorStatusPacketPubSubType newInstance()
   {
      return new BehaviorStatusPacketPubSubType();
   }
}
