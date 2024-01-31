package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerActionMessage" defined in "FootstepPlannerActionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerActionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerActionMessage_.idl instead.
*
*/
public class FootstepPlannerActionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlannerActionMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlannerActionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b131cd3686c977ad099934f2fa74cee7c68bf1ac87464955476c8f597729d1eb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlannerActionMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRequestedAction());

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRequestedAction(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("requested_action", data.getRequestedAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlannerActionMessage data)
   {
      data.setRequestedAction(ser.read_type_9("requested_action"));   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlannerActionMessage src, toolbox_msgs.msg.dds.FootstepPlannerActionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlannerActionMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlannerActionMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlannerActionMessage src, toolbox_msgs.msg.dds.FootstepPlannerActionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerActionMessagePubSubType newInstance()
   {
      return new FootstepPlannerActionMessagePubSubType();
   }
}
