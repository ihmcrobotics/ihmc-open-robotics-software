package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandPoseActionStateMessage" defined in "HandPoseActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandPoseActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandPoseActionStateMessage_.idl instead.
*
*/
public class HandPoseActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandPoseActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandPoseActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4eade3c08fe8f2a8d438b60dd52fa1d06750ad7e644351476c290259109d21aa";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandPoseActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getGoalChestTransformToWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getForce(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getTorque(), current_alignment);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getGoalChestTransformToWorld(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getForce(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getTorque(), cdr);
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getSolutionQuality());

   }

   public static void read(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getGoalChestTransformToWorld(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getForce(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getTorque(), cdr);	
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	data.getJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_a("goal_chest_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getGoalChestTransformToWorld());

      ser.write_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.write_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

      ser.write_type_f("joint_angles", data.getJointAngles());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandPoseActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_a("goal_chest_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getGoalChestTransformToWorld());

      ser.read_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.read_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

      ser.read_type_f("joint_angles", data.getJointAngles());
      data.setSolutionQuality(ser.read_type_6("solution_quality"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandPoseActionStateMessage src, behavior_msgs.msg.dds.HandPoseActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandPoseActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.HandPoseActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandPoseActionStateMessage src, behavior_msgs.msg.dds.HandPoseActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandPoseActionStateMessagePubSubType newInstance()
   {
      return new HandPoseActionStateMessagePubSubType();
   }
}
