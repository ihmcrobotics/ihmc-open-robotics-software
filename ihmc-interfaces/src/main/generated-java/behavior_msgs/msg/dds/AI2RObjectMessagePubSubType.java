package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RObjectMessage" defined in "AI2RObjectMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RObjectMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RObjectMessage_.idl instead.
*
*/
public class AI2RObjectMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RObjectMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RObjectMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ce195469ac061c6fe0438843f7f93ecc92651aba90fc32c3184e1676754b63b2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RObjectMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RObjectMessage data) throws java.io.IOException
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
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RObjectMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RObjectMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectName().length() + 1;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPoseInWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getObjectName().length() <= 255)
      cdr.write_type_d(data.getObjectName());else
          throw new RuntimeException("object_name field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPoseInWorld(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.AI2RObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getObjectName());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPoseInWorld(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RObjectMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("object_name", data.getObjectName());
      ser.write_type_a("object_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPoseInWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RObjectMessage data)
   {
      ser.read_type_d("object_name", data.getObjectName());
      ser.read_type_a("object_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPoseInWorld());

   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RObjectMessage src, behavior_msgs.msg.dds.AI2RObjectMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RObjectMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RObjectMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RObjectMessage src, behavior_msgs.msg.dds.AI2RObjectMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RObjectMessagePubSubType newInstance()
   {
      return new AI2RObjectMessagePubSubType();
   }
}
