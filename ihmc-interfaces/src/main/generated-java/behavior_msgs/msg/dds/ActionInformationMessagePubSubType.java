package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionInformationMessage" defined in "ActionInformationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionInformationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionInformationMessage_.idl instead.
*
*/
public class ActionInformationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionInformationMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionInformationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c9d9a67d0c4e4bfd6cd0f2f9c5418d0c3c0936eda03c5194784b34c6f5e3a740";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionInformationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionInformationMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionInformationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionInformationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionInformationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getActionIndex());

   }

   public static void read(behavior_msgs.msg.dds.ActionInformationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setActionIndex(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionInformationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("action_index", data.getActionIndex());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionInformationMessage data)
   {
      data.setActionIndex(ser.read_type_4("action_index"));   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionInformationMessage src, behavior_msgs.msg.dds.ActionInformationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionInformationMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionInformationMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionInformationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionInformationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionInformationMessage src, behavior_msgs.msg.dds.ActionInformationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionInformationMessagePubSubType newInstance()
   {
      return new ActionInformationMessagePubSubType();
   }
}
