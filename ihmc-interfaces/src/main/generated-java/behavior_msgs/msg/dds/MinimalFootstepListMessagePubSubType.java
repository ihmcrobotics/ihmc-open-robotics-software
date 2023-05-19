package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MinimalFootstepListMessage" defined in "MinimalFootstepListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MinimalFootstepListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MinimalFootstepListMessage_.idl instead.
*
*/
public class MinimalFootstepListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.MinimalFootstepListMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::MinimalFootstepListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "19a1c2151bad5d8c3e94b4dfe2e15ef09b035f5d16b1f1ea51dab241ccb44377";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.MinimalFootstepListMessage data) throws java.io.IOException
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

      for(int i0 = 0; i0 < (200); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.MinimalFootstepListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.MinimalFootstepListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      for(int i0 = 0; i0 < data.getMinimalFootsteps().length; ++i0)
      {
              current_alignment += behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType.getCdrSerializedSize(data.getMinimalFootsteps()[i0], current_alignment);
      }
      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getMinimalFootsteps().length; ++i0)
      {
        	behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType.write(data.getMinimalFootsteps()[i0], cdr);		
      }
   }

   public static void read(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getMinimalFootsteps().length; ++i0)
      {
        	behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType.read(data.getMinimalFootsteps()[i0], cdr);	
      }
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("minimal_footsteps", new behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType(), data.getMinimalFootsteps());   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.MinimalFootstepListMessage data)
   {
      ser.read_type_f("minimal_footsteps", new behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType(), data.getMinimalFootsteps());   }

   public static void staticCopy(behavior_msgs.msg.dds.MinimalFootstepListMessage src, behavior_msgs.msg.dds.MinimalFootstepListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.MinimalFootstepListMessage createData()
   {
      return new behavior_msgs.msg.dds.MinimalFootstepListMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.MinimalFootstepListMessage src, behavior_msgs.msg.dds.MinimalFootstepListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MinimalFootstepListMessagePubSubType newInstance()
   {
      return new MinimalFootstepListMessagePubSubType();
   }
}
