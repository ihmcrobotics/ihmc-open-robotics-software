package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PelvisHeightPitchActionDefinitionMessage" defined in "PelvisHeightPitchActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PelvisHeightPitchActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PelvisHeightPitchActionDefinitionMessage_.idl instead.
*
*/
public class PelvisHeightPitchActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::PelvisHeightPitchActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0475b64bffbbda2c35fce26344b24cf9ec82572e18ffe492fdc61b43fb40a2c9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getActionDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParentFrame().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrame().get(i0).length() + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getPelvisTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.write(data.getActionDefinition(), cdr);
      if(data.getParentFrame().size() <= 1000)
      cdr.write_type_e(data.getParentFrame());else
          throw new RuntimeException("parent_frame field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getPelvisTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

   }

   public static void read(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.read(data.getActionDefinition(), cdr);	
      cdr.read_type_e(data.getParentFrame());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getPelvisTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      ser.write_type_e("parent_frame", data.getParentFrame());
      ser.write_type_a("pelvis_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getPelvisTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data)
   {
      ser.read_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      ser.read_type_e("parent_frame", data.getParentFrame());
      ser.read_type_a("pelvis_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getPelvisTransformToParent());

      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage src, behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage src, behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PelvisHeightPitchActionDefinitionMessagePubSubType newInstance()
   {
      return new PelvisHeightPitchActionDefinitionMessagePubSubType();
   }
}
