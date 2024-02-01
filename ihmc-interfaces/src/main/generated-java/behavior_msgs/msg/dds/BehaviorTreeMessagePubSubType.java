package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeMessage" defined in "BehaviorTreeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeMessage_.idl instead.
*
*/
public class BehaviorTreeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b72f0625f45ae2a5a81152eb30b804ef377555294879ce354a37a47d1f9705af";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 2048; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeMessagePubSubType.getCdrSerializedSize(data.getNodes().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getNodes().size() <= 2048)
      cdr.write_type_e(data.getNodes());else
          throw new RuntimeException("nodes field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getNodes());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("nodes", data.getNodes());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeMessage data)
   {
      ser.read_type_e("nodes", data.getNodes());
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeMessage src, behavior_msgs.msg.dds.BehaviorTreeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeMessage src, behavior_msgs.msg.dds.BehaviorTreeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeMessagePubSubType newInstance()
   {
      return new BehaviorTreeMessagePubSubType();
   }
}
