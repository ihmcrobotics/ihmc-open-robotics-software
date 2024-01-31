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
   		return "ad82238a8626079c764930020585ae934bab1ad1007d80d69047cba2fce8e41a";
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getMinimalFootsteps().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType.getCdrSerializedSize(data.getMinimalFootsteps().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getMinimalFootsteps().size() <= 200)
      cdr.write_type_e(data.getMinimalFootsteps());else
          throw new RuntimeException("minimal_footsteps field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getMinimalFootsteps());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.MinimalFootstepListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("minimal_footsteps", data.getMinimalFootsteps());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.MinimalFootstepListMessage data)
   {
      ser.read_type_e("minimal_footsteps", data.getMinimalFootsteps());
   }

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
