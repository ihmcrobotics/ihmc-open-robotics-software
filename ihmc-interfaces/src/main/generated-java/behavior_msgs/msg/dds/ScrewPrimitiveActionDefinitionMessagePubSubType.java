package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ScrewPrimitiveActionDefinitionMessage" defined in "ScrewPrimitiveActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ScrewPrimitiveActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ScrewPrimitiveActionDefinitionMessage_.idl instead.
*
*/
public class ScrewPrimitiveActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ScrewPrimitiveActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0e0905e191d793c01b72234cb29f0ec32de640c5512b09f456996d3425d78dce";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectFrameName().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getScrewAxisPose(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getWrenchContactPose(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getRobotSide());

      if(data.getObjectFrameName().length() <= 255)
      cdr.write_type_d(data.getObjectFrameName());else
          throw new RuntimeException("object_frame_name field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getScrewAxisPose(), cdr);
      cdr.write_type_6(data.getTranslation());

      cdr.write_type_6(data.getRotation());

      cdr.write_type_6(data.getMaxLinearVelocity());

      cdr.write_type_6(data.getMaxAngularVelocity());

      cdr.write_type_6(data.getMaxForce());

      cdr.write_type_6(data.getMaxTorque());

      cdr.write_type_6(data.getLinearPositionWeight());

      cdr.write_type_6(data.getAngularPositionWeight());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getWrenchContactPose(), cdr);
      cdr.write_type_7(data.getHoldPoseInWorld());

   }

   public static void read(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_d(data.getObjectFrameName());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getScrewAxisPose(), cdr);	
      data.setTranslation(cdr.read_type_6());
      	
      data.setRotation(cdr.read_type_6());
      	
      data.setMaxLinearVelocity(cdr.read_type_6());
      	
      data.setMaxAngularVelocity(cdr.read_type_6());
      	
      data.setMaxForce(cdr.read_type_6());
      	
      data.setMaxTorque(cdr.read_type_6());
      	
      data.setLinearPositionWeight(cdr.read_type_6());
      	
      data.setAngularPositionWeight(cdr.read_type_6());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getWrenchContactPose(), cdr);	
      data.setHoldPoseInWorld(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_d("object_frame_name", data.getObjectFrameName());
      ser.write_type_a("screw_axis_pose", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getScrewAxisPose());

      ser.write_type_6("translation", data.getTranslation());
      ser.write_type_6("rotation", data.getRotation());
      ser.write_type_6("max_linear_velocity", data.getMaxLinearVelocity());
      ser.write_type_6("max_angular_velocity", data.getMaxAngularVelocity());
      ser.write_type_6("max_force", data.getMaxForce());
      ser.write_type_6("max_torque", data.getMaxTorque());
      ser.write_type_6("linear_position_weight", data.getLinearPositionWeight());
      ser.write_type_6("angular_position_weight", data.getAngularPositionWeight());
      ser.write_type_a("wrench_contact_pose", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getWrenchContactPose());

      ser.write_type_7("hold_pose_in_world", data.getHoldPoseInWorld());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_d("object_frame_name", data.getObjectFrameName());
      ser.read_type_a("screw_axis_pose", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getScrewAxisPose());

      data.setTranslation(ser.read_type_6("translation"));
      data.setRotation(ser.read_type_6("rotation"));
      data.setMaxLinearVelocity(ser.read_type_6("max_linear_velocity"));
      data.setMaxAngularVelocity(ser.read_type_6("max_angular_velocity"));
      data.setMaxForce(ser.read_type_6("max_force"));
      data.setMaxTorque(ser.read_type_6("max_torque"));
      data.setLinearPositionWeight(ser.read_type_6("linear_position_weight"));
      data.setAngularPositionWeight(ser.read_type_6("angular_position_weight"));
      ser.read_type_a("wrench_contact_pose", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getWrenchContactPose());

      data.setHoldPoseInWorld(ser.read_type_7("hold_pose_in_world"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage src, behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage src, behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ScrewPrimitiveActionDefinitionMessagePubSubType newInstance()
   {
      return new ScrewPrimitiveActionDefinitionMessagePubSubType();
   }
}
