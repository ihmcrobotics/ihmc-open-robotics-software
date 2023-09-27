package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BodyPartPoseStatusMessage" defined in "BodyPartPoseStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BodyPartPoseStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BodyPartPoseStatusMessage_.idl instead.
*
*/
public class BodyPartPoseStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BodyPartPoseStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BodyPartPoseStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "626db517eac0a1dcd0ed7519aad68fbc2701f987befa3f2b465a948e3f1f6c17";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BodyPartPoseStatusMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParentFrame().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrame().get(i0).length() + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToParent(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getActionIndex());

      cdr.write_type_7(data.getCurrentAndConcurrent());

      if(data.getParentFrame().size() <= 1000)
      cdr.write_type_e(data.getParentFrame());else
          throw new RuntimeException("parent_frame field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToParent(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setActionIndex(cdr.read_type_4());
      	
      data.setCurrentAndConcurrent(cdr.read_type_7());
      	
      cdr.read_type_e(data.getParentFrame());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToParent(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("action_index", data.getActionIndex());
      ser.write_type_7("current_and_concurrent", data.getCurrentAndConcurrent());
      ser.write_type_e("parent_frame", data.getParentFrame());
      ser.write_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BodyPartPoseStatusMessage data)
   {
      data.setActionIndex(ser.read_type_4("action_index"));
      data.setCurrentAndConcurrent(ser.read_type_7("current_and_concurrent"));
      ser.read_type_e("parent_frame", data.getParentFrame());
      ser.read_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

   }

   public static void staticCopy(behavior_msgs.msg.dds.BodyPartPoseStatusMessage src, behavior_msgs.msg.dds.BodyPartPoseStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BodyPartPoseStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.BodyPartPoseStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BodyPartPoseStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BodyPartPoseStatusMessage src, behavior_msgs.msg.dds.BodyPartPoseStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BodyPartPoseStatusMessagePubSubType newInstance()
   {
      return new BodyPartPoseStatusMessagePubSubType();
   }
}
