package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DoorTraversalStateMessage" defined in "DoorTraversalStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DoorTraversalStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DoorTraversalStateMessage_.idl instead.
*
*/
public class DoorTraversalStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.DoorTraversalStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::DoorTraversalStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b5fc645714f37f0eead27ba3b2043e6e78c456658cc5a35a211769d5b697a12a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.DoorTraversalStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.DoorTraversalStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.DoorTraversalStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.DoorTraversalStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.DoorTraversalStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_7(data.getRetryingPullDoorNotification());

   }

   public static void read(behavior_msgs.msg.dds.DoorTraversalStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setRetryingPullDoorNotification(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.DoorTraversalStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_7("retrying_pull_door_notification", data.getRetryingPullDoorNotification());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.DoorTraversalStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.DoorTraversalDefinitionMessagePubSubType(), data.getDefinition());

      data.setRetryingPullDoorNotification(ser.read_type_7("retrying_pull_door_notification"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.DoorTraversalStateMessage src, behavior_msgs.msg.dds.DoorTraversalStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.DoorTraversalStateMessage createData()
   {
      return new behavior_msgs.msg.dds.DoorTraversalStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.DoorTraversalStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.DoorTraversalStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.DoorTraversalStateMessage src, behavior_msgs.msg.dds.DoorTraversalStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DoorTraversalStateMessagePubSubType newInstance()
   {
      return new DoorTraversalStateMessagePubSubType();
   }
}
