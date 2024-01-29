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
   		return "664a2d13de78599aa23f11f8b027e662e37d442c2ae0b378089f5440894a47fc";
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDescription().length() + 1;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getJsonFileName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDescription().length() <= 255)
      cdr.write_type_d(data.getDescription());else
          throw new RuntimeException("description field exceeds the maximum length");

      cdr.write_type_3(data.getNumberOfChildren());

      if(data.getJsonFileName().length() <= 255)
      cdr.write_type_d(data.getJsonFileName());else
          throw new RuntimeException("json_file_name field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getDescription());	
      data.setNumberOfChildren(cdr.read_type_3());
      	
      cdr.read_type_d(data.getJsonFileName());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("description", data.getDescription());
      ser.write_type_3("number_of_children", data.getNumberOfChildren());
      ser.write_type_d("json_file_name", data.getJsonFileName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage data)
   {
      ser.read_type_d("description", data.getDescription());
      data.setNumberOfChildren(ser.read_type_3("number_of_children"));
      ser.read_type_d("json_file_name", data.getJsonFileName());
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
