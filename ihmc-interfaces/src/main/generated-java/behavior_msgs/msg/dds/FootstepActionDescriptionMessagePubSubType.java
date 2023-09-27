package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepActionDescriptionMessage" defined in "FootstepActionDescriptionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepActionDescriptionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepActionDescriptionMessage_.idl instead.
*
*/
public class FootstepActionDescriptionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepActionDescriptionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepActionDescriptionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "34e4a8c6bd82eb300363569b5db5569dedd408109f021b5840b034f14c11a1d7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepActionDescriptionMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getSolePose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getSolePose(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getSolePose(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("sole_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSolePose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepActionDescriptionMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("sole_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSolePose());

   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepActionDescriptionMessage src, behavior_msgs.msg.dds.FootstepActionDescriptionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepActionDescriptionMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepActionDescriptionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepActionDescriptionMessage src, behavior_msgs.msg.dds.FootstepActionDescriptionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepActionDescriptionMessagePubSubType newInstance()
   {
      return new FootstepActionDescriptionMessagePubSubType();
   }
}
