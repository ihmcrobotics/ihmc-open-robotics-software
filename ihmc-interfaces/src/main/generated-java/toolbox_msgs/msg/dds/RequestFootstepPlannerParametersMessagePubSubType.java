package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RequestFootstepPlannerParametersMessage" defined in "RequestFootstepPlannerParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RequestFootstepPlannerParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RequestFootstepPlannerParametersMessage_.idl instead.
*
*/
public class RequestFootstepPlannerParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::RequestFootstepPlannerParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d6a067283b8d8fd9c0c4469c8486e9884809eb2878da60fae1d54fccfb2bf445";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getUnusedPlaceholderField());

   }

   public static void read(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setUnusedPlaceholderField(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("unused_placeholder_field", data.getUnusedPlaceholderField());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data)
   {
      data.setUnusedPlaceholderField(ser.read_type_7("unused_placeholder_field"));   }

   public static void staticCopy(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage src, toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage createData()
   {
      return new toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage src, toolbox_msgs.msg.dds.RequestFootstepPlannerParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestFootstepPlannerParametersMessagePubSubType newInstance()
   {
      return new RequestFootstepPlannerParametersMessagePubSubType();
   }
}
