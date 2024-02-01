package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanActionFootstepDefinitionMessage" defined in "FootstepPlanActionFootstepDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanActionFootstepDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanActionFootstepDefinitionMessage_.idl instead.
*
*/
public class FootstepPlanActionFootstepDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepPlanActionFootstepDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "aed1cc7a2eeba167abf5adf9fb1d05929ddd38464826922995b241353fbf1bdf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getSolePose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getSolePose(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getSolePose(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("sole_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSolePose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("sole_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSolePose());

   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage src, behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage src, behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanActionFootstepDefinitionMessagePubSubType newInstance()
   {
      return new FootstepPlanActionFootstepDefinitionMessagePubSubType();
   }
}
