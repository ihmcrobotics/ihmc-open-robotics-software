package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerStatusMessage" defined in "FootstepPlannerStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerStatusMessage_.idl instead.
*
*/
public class FootstepPlannerStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlannerStatusMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlannerStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "952c8097725a04cfb4957b3ca280c4461bada85bd398a1a53d13b11da017f0e8";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFootstepPlannerStatus());

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFootstepPlannerStatus(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("footstep_planner_status", data.getFootstepPlannerStatus());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFootstepPlannerStatus(ser.read_type_9("footstep_planner_status"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage src, toolbox_msgs.msg.dds.FootstepPlannerStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlannerStatusMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlannerStatusMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlannerStatusMessage src, toolbox_msgs.msg.dds.FootstepPlannerStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerStatusMessagePubSubType newInstance()
   {
      return new FootstepPlannerStatusMessagePubSubType();
   }
}
