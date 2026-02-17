package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PersistentDetectionStatusMessage" defined in "PersistentDetectionStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PersistentDetectionStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PersistentDetectionStatusMessage_.idl instead.
*
*/
public class PersistentDetectionStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.PersistentDetectionStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::PersistentDetectionStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "432ccbf0943506ebb74ca9b2d88e22d3dbd008f9a1a66b4e8e2d19129220e97b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.PersistentDetectionStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDetectionType().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectClass().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToWorld(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToCamera(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getId());

      if(data.getDetectionType().length() <= 255)
      cdr.write_type_d(data.getDetectionType());else
          throw new RuntimeException("detection_type field exceeds the maximum length: %d > %d".formatted(data.getDetectionType().length(), 255));

      if(data.getObjectClass().length() <= 255)
      cdr.write_type_d(data.getObjectClass());else
          throw new RuntimeException("object_class field exceeds the maximum length: %d > %d".formatted(data.getObjectClass().length(), 255));

      cdr.write_type_6(data.getDecayingFrequency());

      cdr.write_type_2(data.getHistorySize());

      cdr.write_type_7(data.getIsStable());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToWorld(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToCamera(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setId(cdr.read_type_2());
      	
      cdr.read_type_d(data.getDetectionType());	
      cdr.read_type_d(data.getObjectClass());	
      data.setDecayingFrequency(cdr.read_type_6());
      	
      data.setHistorySize(cdr.read_type_2());
      	
      data.setIsStable(cdr.read_type_7());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToWorld(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToCamera(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("id", data.getId());
      ser.write_type_d("detection_type", data.getDetectionType());
      ser.write_type_d("object_class", data.getObjectClass());
      ser.write_type_6("decaying_frequency", data.getDecayingFrequency());
      ser.write_type_2("history_size", data.getHistorySize());
      ser.write_type_7("is_stable", data.getIsStable());
      ser.write_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.write_type_a("transform_to_camera", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToCamera());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.PersistentDetectionStatusMessage data)
   {
      data.setId(ser.read_type_2("id"));
      ser.read_type_d("detection_type", data.getDetectionType());
      ser.read_type_d("object_class", data.getObjectClass());
      data.setDecayingFrequency(ser.read_type_6("decaying_frequency"));
      data.setHistorySize(ser.read_type_2("history_size"));
      data.setIsStable(ser.read_type_7("is_stable"));
      ser.read_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.read_type_a("transform_to_camera", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToCamera());

   }

   public static void staticCopy(behavior_msgs.msg.dds.PersistentDetectionStatusMessage src, behavior_msgs.msg.dds.PersistentDetectionStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.PersistentDetectionStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.PersistentDetectionStatusMessage src, behavior_msgs.msg.dds.PersistentDetectionStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PersistentDetectionStatusMessagePubSubType newInstance()
   {
      return new PersistentDetectionStatusMessagePubSubType();
   }
}
