package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StaticRelativeSceneNodeMessage" defined in "StaticRelativeSceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StaticRelativeSceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StaticRelativeSceneNodeMessage_.idl instead.
*
*/
public class StaticRelativeSceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::StaticRelativeSceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "02444c03ee009ef16044010b5833fb3417e09115405d899b8039b86808b4d720";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.getCdrSerializedSize(data.getPredefinedRigidBodySceneNode(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.write(data.getPredefinedRigidBodySceneNode(), cdr);
      cdr.write_type_5(data.getDistanceToDisableTracking());

      cdr.write_type_5(data.getCurrentDistanceToRobot());

   }

   public static void read(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.read(data.getPredefinedRigidBodySceneNode(), cdr);	
      data.setDistanceToDisableTracking(cdr.read_type_5());
      	
      data.setCurrentDistanceToRobot(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("predefined_rigid_body_scene_node", new perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType(), data.getPredefinedRigidBodySceneNode());

      ser.write_type_5("distance_to_disable_tracking", data.getDistanceToDisableTracking());
      ser.write_type_5("current_distance_to_robot", data.getCurrentDistanceToRobot());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data)
   {
      ser.read_type_a("predefined_rigid_body_scene_node", new perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType(), data.getPredefinedRigidBodySceneNode());

      data.setDistanceToDisableTracking(ser.read_type_5("distance_to_disable_tracking"));
      data.setCurrentDistanceToRobot(ser.read_type_5("current_distance_to_robot"));
   }

   public static void staticCopy(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage src, perception_msgs.msg.dds.StaticRelativeSceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.StaticRelativeSceneNodeMessage createData()
   {
      return new perception_msgs.msg.dds.StaticRelativeSceneNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.StaticRelativeSceneNodeMessage src, perception_msgs.msg.dds.StaticRelativeSceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StaticRelativeSceneNodeMessagePubSubType newInstance()
   {
      return new StaticRelativeSceneNodeMessagePubSubType();
   }
}
