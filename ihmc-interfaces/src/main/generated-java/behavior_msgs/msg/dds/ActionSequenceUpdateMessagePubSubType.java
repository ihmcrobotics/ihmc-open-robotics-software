package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionSequenceUpdateMessage" defined in "ActionSequenceUpdateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionSequenceUpdateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionSequenceUpdateMessage_.idl instead.
*
*/
public class ActionSequenceUpdateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionSequenceUpdateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionSequenceUpdateMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionSequenceUpdateMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceUpdateUuid());

      cdr.write_type_4(data.getSequenceSize());

   }

   public static void read(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceUpdateUuid(cdr.read_type_12());
      	
      data.setSequenceSize(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_update_uuid", data.getSequenceUpdateUuid());
      ser.write_type_4("sequence_size", data.getSequenceSize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionSequenceUpdateMessage data)
   {
      data.setSequenceUpdateUuid(ser.read_type_12("sequence_update_uuid"));
      data.setSequenceSize(ser.read_type_4("sequence_size"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionSequenceUpdateMessage src, behavior_msgs.msg.dds.ActionSequenceUpdateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionSequenceUpdateMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionSequenceUpdateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionSequenceUpdateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionSequenceUpdateMessage src, behavior_msgs.msg.dds.ActionSequenceUpdateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionSequenceUpdateMessagePubSubType newInstance()
   {
      return new ActionSequenceUpdateMessagePubSubType();
   }
}
