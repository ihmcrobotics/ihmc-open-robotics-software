package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AbilityHandActionStateMessage" defined in "AbilityHandActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AbilityHandActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AbilityHandActionStateMessage_.idl instead.
*
*/
public class AbilityHandActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AbilityHandActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AbilityHandActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7b14e1d790c7c6e8485134ba58a5172a3752ea9db3f1a894cbd0e08cb2fdc8cc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AbilityHandActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AbilityHandActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      for(int i0 = 0; i0 < data.getCurrentFingerPositions().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentFingerPositions()[i0]);	
      }

      for(int i0 = 0; i0 < data.getDesiredFingerPositions().length; ++i0)
      {
        	cdr.write_type_5(data.getDesiredFingerPositions()[i0]);	
      }

   }

   public static void read(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      for(int i0 = 0; i0 < data.getCurrentFingerPositions().length; ++i0)
      {
        	data.getCurrentFingerPositions()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getDesiredFingerPositions().length; ++i0)
      {
        	data.getDesiredFingerPositions()[i0] = cdr.read_type_5();
        	
      }
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_f("current_finger_positions", data.getCurrentFingerPositions());
      ser.write_type_f("desired_finger_positions", data.getDesiredFingerPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AbilityHandActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.AbilityHandActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_f("current_finger_positions", data.getCurrentFingerPositions());
      ser.read_type_f("desired_finger_positions", data.getDesiredFingerPositions());
   }

   public static void staticCopy(behavior_msgs.msg.dds.AbilityHandActionStateMessage src, behavior_msgs.msg.dds.AbilityHandActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AbilityHandActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.AbilityHandActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AbilityHandActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AbilityHandActionStateMessage src, behavior_msgs.msg.dds.AbilityHandActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AbilityHandActionStateMessagePubSubType newInstance()
   {
      return new AbilityHandActionStateMessagePubSubType();
   }
}
