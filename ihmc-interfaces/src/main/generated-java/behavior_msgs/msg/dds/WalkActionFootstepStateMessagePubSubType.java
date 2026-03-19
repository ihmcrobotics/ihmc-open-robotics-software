package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkActionFootstepStateMessage" defined in "WalkActionFootstepStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkActionFootstepStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkActionFootstepStateMessage_.idl instead.
*
*/
public class WalkActionFootstepStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WalkActionFootstepStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WalkActionFootstepStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f5103db649ad5391b5d297908440c2cac0ced3ab2a85dfa50b2738432cae5ca4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WalkActionFootstepStateMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getUnusedPlaceholderField());

   }

   public static void read(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setUnusedPlaceholderField(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("unused_placeholder_field", data.getUnusedPlaceholderField());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WalkActionFootstepStateMessage data)
   {
      data.setUnusedPlaceholderField(ser.read_type_7("unused_placeholder_field"));   }

   public static void staticCopy(behavior_msgs.msg.dds.WalkActionFootstepStateMessage src, behavior_msgs.msg.dds.WalkActionFootstepStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WalkActionFootstepStateMessage createData()
   {
      return new behavior_msgs.msg.dds.WalkActionFootstepStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WalkActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WalkActionFootstepStateMessage src, behavior_msgs.msg.dds.WalkActionFootstepStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkActionFootstepStateMessagePubSubType newInstance()
   {
      return new WalkActionFootstepStateMessagePubSubType();
   }
}
