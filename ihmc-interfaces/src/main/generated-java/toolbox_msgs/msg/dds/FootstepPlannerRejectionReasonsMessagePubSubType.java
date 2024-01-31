package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerRejectionReasonsMessage" defined in "FootstepPlannerRejectionReasonsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerRejectionReasonsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerRejectionReasonsMessage_.idl instead.
*
*/
public class FootstepPlannerRejectionReasonsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlannerRejectionReasonsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f4a37c667b6a6655728032343b27c1c4cb3882c324dd68df94cb94706e891417";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 30; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRejectionReasons().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessagePubSubType.getCdrSerializedSize(data.getRejectionReasons().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getRejectionReasons().size() <= 30)
      cdr.write_type_e(data.getRejectionReasons());else
          throw new RuntimeException("rejection_reasons field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getRejectionReasons());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("rejection_reasons", data.getRejectionReasons());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data)
   {
      ser.read_type_e("rejection_reasons", data.getRejectionReasons());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage src, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage src, toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerRejectionReasonsMessagePubSubType newInstance()
   {
      return new FootstepPlannerRejectionReasonsMessagePubSubType();
   }
}
