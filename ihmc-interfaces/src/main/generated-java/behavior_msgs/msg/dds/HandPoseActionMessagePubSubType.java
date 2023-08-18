package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandPoseActionMessage" defined in "HandPoseActionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandPoseActionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandPoseActionMessage_.idl instead.
*
*/
public class HandPoseActionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandPoseActionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandPoseActionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4d82e26c7dbbf825ead7d39d8b29cde338ced593754ca0e38789ddb227b5dbf0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandPoseActionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandPoseActionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getCdrSerializedSize(data.getActionInformation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParentFrame().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrame().get(i0).length() + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.write(data.getActionInformation(), cdr);
      cdr.write_type_9(data.getRobotSide());

      if(data.getParentFrame().size() <= 1000)
      cdr.write_type_e(data.getParentFrame());else
          throw new RuntimeException("parent_frame field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

   }

   public static void read(behavior_msgs.msg.dds.HandPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.read(data.getActionInformation(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getParentFrame());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandPoseActionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("parent_frame", data.getParentFrame());
      ser.write_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandPoseActionMessage data)
   {
      ser.read_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("parent_frame", data.getParentFrame());
      ser.read_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandPoseActionMessage src, behavior_msgs.msg.dds.HandPoseActionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandPoseActionMessage createData()
   {
      return new behavior_msgs.msg.dds.HandPoseActionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandPoseActionMessage src, behavior_msgs.msg.dds.HandPoseActionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandPoseActionMessagePubSubType newInstance()
   {
      return new HandPoseActionMessagePubSubType();
   }
}
