package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RCommandMessage" defined in "AI2RCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RCommandMessage_.idl instead.
*
*/
public class AI2RCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RCommandMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "99233c6f23e4b4243e488c6c4f0f53065fedb240510e78b64dfcb1bbf314db3e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RCommandMessage data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getBehaviorToExecute().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getGotoReferenceFrameName().length() + 1;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGotoGoalStancePoint(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGotoGoalFocalPoint(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getBehaviorToExecute().length() <= 255)
      cdr.write_type_d(data.getBehaviorToExecute());else
          throw new RuntimeException("behavior_to_execute field exceeds the maximum length");

      if(data.getGotoReferenceFrameName().length() <= 255)
      cdr.write_type_d(data.getGotoReferenceFrameName());else
          throw new RuntimeException("goto_reference_frame_name field exceeds the maximum length");

      geometry_msgs.msg.dds.PointPubSubType.write(data.getGotoGoalStancePoint(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGotoGoalFocalPoint(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getBehaviorToExecute());	
      cdr.read_type_d(data.getGotoReferenceFrameName());	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGotoGoalStancePoint(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGotoGoalFocalPoint(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("behavior_to_execute", data.getBehaviorToExecute());
      ser.write_type_d("goto_reference_frame_name", data.getGotoReferenceFrameName());
      ser.write_type_a("goto_goal_stance_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGotoGoalStancePoint());

      ser.write_type_a("goto_goal_focal_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGotoGoalFocalPoint());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RCommandMessage data)
   {
      ser.read_type_d("behavior_to_execute", data.getBehaviorToExecute());
      ser.read_type_d("goto_reference_frame_name", data.getGotoReferenceFrameName());
      ser.read_type_a("goto_goal_stance_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGotoGoalStancePoint());

      ser.read_type_a("goto_goal_focal_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getGotoGoalFocalPoint());

   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RCommandMessage src, behavior_msgs.msg.dds.AI2RCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RCommandMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RCommandMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RCommandMessage src, behavior_msgs.msg.dds.AI2RCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RCommandMessagePubSubType newInstance()
   {
      return new AI2RCommandMessagePubSubType();
   }
}
