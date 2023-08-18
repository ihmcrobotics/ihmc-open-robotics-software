package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionExecutionStatusMessage" defined in "ActionExecutionStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionExecutionStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionExecutionStatusMessage_.idl instead.
*
*/
public class ActionExecutionStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionExecutionStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionExecutionStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "51e02ace02692b7d736afe1275673ce00e73addb9ab10c188646f7048ec6dab2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionExecutionStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getNominalExecutionDuration());

      cdr.write_type_6(data.getElapsedExecutionTime());

   }

   public static void read(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setNominalExecutionDuration(cdr.read_type_6());
      	
      data.setElapsedExecutionTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("nominal_execution_duration", data.getNominalExecutionDuration());
      ser.write_type_6("elapsed_execution_time", data.getElapsedExecutionTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionExecutionStatusMessage data)
   {
      data.setNominalExecutionDuration(ser.read_type_6("nominal_execution_duration"));
      data.setElapsedExecutionTime(ser.read_type_6("elapsed_execution_time"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionExecutionStatusMessage src, behavior_msgs.msg.dds.ActionExecutionStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionExecutionStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionExecutionStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionExecutionStatusMessage src, behavior_msgs.msg.dds.ActionExecutionStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionExecutionStatusMessagePubSubType newInstance()
   {
      return new ActionExecutionStatusMessagePubSubType();
   }
}
