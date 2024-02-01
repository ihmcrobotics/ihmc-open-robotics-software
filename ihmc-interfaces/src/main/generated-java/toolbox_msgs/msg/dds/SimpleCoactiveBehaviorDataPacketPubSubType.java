package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SimpleCoactiveBehaviorDataPacket" defined in "SimpleCoactiveBehaviorDataPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SimpleCoactiveBehaviorDataPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SimpleCoactiveBehaviorDataPacket_.idl instead.
*
*/
public class SimpleCoactiveBehaviorDataPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::SimpleCoactiveBehaviorDataPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "45212f140cbd4641b0ffe5ced31cbaed79c8a72aa6fde2badc2cf26648bf11b9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data) throws java.io.IOException
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
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getKey().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getKey().length() <= 255)
      cdr.write_type_d(data.getKey());else
          throw new RuntimeException("key field exceeds the maximum length");

      cdr.write_type_6(data.getValue());

   }

   public static void read(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getKey());	
      data.setValue(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("key", data.getKey());
      ser.write_type_6("value", data.getValue());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("key", data.getKey());
      data.setValue(ser.read_type_6("value"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket src, toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket createData()
   {
      return new toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket src, toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SimpleCoactiveBehaviorDataPacketPubSubType newInstance()
   {
      return new SimpleCoactiveBehaviorDataPacketPubSubType();
   }
}
