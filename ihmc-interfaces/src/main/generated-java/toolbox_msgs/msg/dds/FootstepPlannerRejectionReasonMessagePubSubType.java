package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerRejectionReasonMessage" defined in "FootstepPlannerRejectionReasonMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerRejectionReasonMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerRejectionReasonMessage_.idl instead.
*
*/
public class FootstepPlannerRejectionReasonMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlannerRejectionReasonMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ccfdf030966c575dfcfe3bffc184f18c2c75f80eff581164d6440944bc0cec74";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getReason());

      cdr.write_type_5(data.getRejectionPercentage());

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setReason(cdr.read_type_4());
      	
      data.setRejectionPercentage(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("reason", data.getReason());
      ser.write_type_5("rejection_percentage", data.getRejectionPercentage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data)
   {
      data.setReason(ser.read_type_4("reason"));
      data.setRejectionPercentage(ser.read_type_5("rejection_percentage"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage src, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage src, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerRejectionReasonMessagePubSubType newInstance()
   {
      return new FootstepPlannerRejectionReasonMessagePubSubType();
   }
}
