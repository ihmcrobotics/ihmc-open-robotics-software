package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepActionDefinitionMessage" defined in "FootstepActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepActionDefinitionMessage_.idl instead.
*
*/
public class FootstepActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0ee39974529b1c2b3842abb6e804bc8db1d8c83894d66ff6ae325000712cbe34";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepActionDefinitionMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getSolePose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getSolePose(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getSolePose(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("sole_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSolePose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepActionDefinitionMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("sole_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSolePose());

   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepActionDefinitionMessage src, behavior_msgs.msg.dds.FootstepActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepActionDefinitionMessage src, behavior_msgs.msg.dds.FootstepActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepActionDefinitionMessagePubSubType newInstance()
   {
      return new FootstepActionDefinitionMessagePubSubType();
   }
}
