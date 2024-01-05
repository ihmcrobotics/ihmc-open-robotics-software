package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanActionDefinitionBasicsMessage" defined in "FootstepPlanActionDefinitionBasicsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanActionDefinitionBasicsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanActionDefinitionBasicsMessage_.idl instead.
*
*/
public class FootstepPlanActionDefinitionBasicsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepPlanActionDefinitionBasicsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d24266a8d78ed430fbe403b9af8cae0df909122883f2a5fa0c49af911b3f7fb6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

   }

   public static void read(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getParentFrameName());	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data)
   {
      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage src, behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage src, behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanActionDefinitionBasicsMessagePubSubType newInstance()
   {
      return new FootstepPlanActionDefinitionBasicsMessagePubSubType();
   }
}
