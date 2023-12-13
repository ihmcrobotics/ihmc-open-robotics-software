package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkActionStateMessage" defined in "WalkActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkActionStateMessage_.idl instead.
*
*/
public class WalkActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WalkActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WalkActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "476c959fb0dc4e490158ee0ba0244340be0d475f28e77d831589e27d5d606753";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WalkActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WalkActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WalkActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_3(data.getTotalNumberOfFootsteps());

      cdr.write_type_3(data.getNumberOfIncompleteFootsteps());

   }

   public static void read(behavior_msgs.msg.dds.WalkActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setTotalNumberOfFootsteps(cdr.read_type_3());
      	
      data.setNumberOfIncompleteFootsteps(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WalkActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_3("total_number_of_footsteps", data.getTotalNumberOfFootsteps());
      ser.write_type_3("number_of_incomplete_footsteps", data.getNumberOfIncompleteFootsteps());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WalkActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType(), data.getDefinition());

      data.setTotalNumberOfFootsteps(ser.read_type_3("total_number_of_footsteps"));
      data.setNumberOfIncompleteFootsteps(ser.read_type_3("number_of_incomplete_footsteps"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.WalkActionStateMessage src, behavior_msgs.msg.dds.WalkActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WalkActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.WalkActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WalkActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WalkActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WalkActionStateMessage src, behavior_msgs.msg.dds.WalkActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkActionStateMessagePubSubType newInstance()
   {
      return new WalkActionStateMessagePubSubType();
   }
}
