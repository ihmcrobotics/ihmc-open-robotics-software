package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeNodeDefinitionMessage" defined in "BehaviorTreeNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeNodeDefinitionMessage_.idl instead.
*
*/
public class BehaviorTreeNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5de090fbb6abf284742220856935c60049ac4335a81197831b63cfafe38b9cd4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.getCdrSerializedSize(data.getConfirmableRequest(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.write(data.getConfirmableRequest(), cdr);
      if(data.getName().length() <= 255)
      cdr.write_type_d(data.getName());else
          throw new RuntimeException("name field exceeds the maximum length");

      cdr.write_type_3(data.getNumberOfChildren());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.read(data.getConfirmableRequest(), cdr);	
      cdr.read_type_d(data.getName());	
      data.setNumberOfChildren(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("confirmable_request", new ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType(), data.getConfirmableRequest());

      ser.write_type_d("name", data.getName());
      ser.write_type_3("number_of_children", data.getNumberOfChildren());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data)
   {
      ser.read_type_a("confirmable_request", new ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType(), data.getConfirmableRequest());

      ser.read_type_d("name", data.getName());
      data.setNumberOfChildren(ser.read_type_3("number_of_children"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage src, behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage src, behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeNodeDefinitionMessagePubSubType newInstance()
   {
      return new BehaviorTreeNodeDefinitionMessagePubSubType();
   }
}
