package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeNodeMessage" defined in "BehaviorTreeNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeNodeMessage_.idl instead.
*
*/
public class BehaviorTreeNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeNodeMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4d1adcc4031b33de6b581967e1755596b4356127ec05de8d0890d51c31783239";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeNodeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLastTickInstant(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getNodeName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getNodeType().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getNumberOfChildren());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLastTickInstant(), cdr);
      if(data.getNodeName().length() <= 255)
      cdr.write_type_d(data.getNodeName());else
          throw new RuntimeException("node_name field exceeds the maximum length");

      if(data.getNodeType().length() <= 255)
      cdr.write_type_d(data.getNodeType());else
          throw new RuntimeException("node_type field exceeds the maximum length");

      cdr.write_type_9(data.getPreviousStatus());

      cdr.write_type_7(data.getHasBeenClocked());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setNumberOfChildren(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLastTickInstant(), cdr);	
      cdr.read_type_d(data.getNodeName());	
      cdr.read_type_d(data.getNodeType());	
      data.setPreviousStatus(cdr.read_type_9());
      	
      data.setHasBeenClocked(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("number_of_children", data.getNumberOfChildren());
      ser.write_type_a("last_tick_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastTickInstant());

      ser.write_type_d("node_name", data.getNodeName());
      ser.write_type_d("node_type", data.getNodeType());
      ser.write_type_9("previous_status", data.getPreviousStatus());
      ser.write_type_7("has_been_clocked", data.getHasBeenClocked());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeNodeMessage data)
   {
      data.setNumberOfChildren(ser.read_type_4("number_of_children"));
      ser.read_type_a("last_tick_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastTickInstant());

      ser.read_type_d("node_name", data.getNodeName());
      ser.read_type_d("node_type", data.getNodeType());
      data.setPreviousStatus(ser.read_type_9("previous_status"));
      data.setHasBeenClocked(ser.read_type_7("has_been_clocked"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeNodeMessage src, behavior_msgs.msg.dds.BehaviorTreeNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeNodeMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeNodeMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeNodeMessage src, behavior_msgs.msg.dds.BehaviorTreeNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeNodeMessagePubSubType newInstance()
   {
      return new BehaviorTreeNodeMessagePubSubType();
   }
}
