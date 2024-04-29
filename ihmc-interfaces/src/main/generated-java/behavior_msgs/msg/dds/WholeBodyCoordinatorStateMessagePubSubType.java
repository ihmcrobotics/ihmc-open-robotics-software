package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyCoordinatorStateMessage" defined in "WholeBodyCoordinatorStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyCoordinatorStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyCoordinatorStateMessage_.idl instead.
*
*/
public class WholeBodyCoordinatorStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WholeBodyCoordinatorStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8577c5501418a6dee3d0e4fba09bafccafbc1d7c8d89ae2d11886391f69ba24d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      cdr.write_type_6(data.getPreviewRequestedTime());

   }

   public static void read(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      data.setPreviewRequestedTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_6("preview_requested_time", data.getPreviewRequestedTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      data.setPreviewRequestedTime(ser.read_type_6("preview_requested_time"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage src, behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage createData()
   {
      return new behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage src, behavior_msgs.msg.dds.WholeBodyCoordinatorStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyCoordinatorStateMessagePubSubType newInstance()
   {
      return new WholeBodyCoordinatorStateMessagePubSubType();
   }
}
