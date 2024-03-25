package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TriggerKickMessage" defined in "TriggerKickMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TriggerKickMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TriggerKickMessage_.idl instead.
*
*/
public class TriggerKickMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.TriggerKickMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::TriggerKickMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "79a156c18de158aaca32e57b52e06d1d08f658115f0d4e04a0c09e454e942e15";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.TriggerKickMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.TriggerKickMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TriggerKickMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TriggerKickMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TriggerKickMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

   }

   public static void read(controller_msgs.msg.dds.TriggerKickMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TriggerKickMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TriggerKickMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));   }

   public static void staticCopy(controller_msgs.msg.dds.TriggerKickMessage src, controller_msgs.msg.dds.TriggerKickMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.TriggerKickMessage createData()
   {
      return new controller_msgs.msg.dds.TriggerKickMessage();
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
   
   public void serialize(controller_msgs.msg.dds.TriggerKickMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.TriggerKickMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.TriggerKickMessage src, controller_msgs.msg.dds.TriggerKickMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TriggerKickMessagePubSubType newInstance()
   {
      return new TriggerKickMessagePubSubType();
   }
}
