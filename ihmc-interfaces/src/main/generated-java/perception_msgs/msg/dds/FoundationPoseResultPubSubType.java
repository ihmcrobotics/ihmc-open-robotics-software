package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FoundationPoseResult" defined in "FoundationPoseResult_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FoundationPoseResult_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FoundationPoseResult_.idl instead.
*
*/
public class FoundationPoseResultPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.FoundationPoseResult>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::FoundationPoseResult_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1713137489585ad8840f7d75f5696274c62da348440dc73c6be70d919ad01273";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.FoundationPoseResult data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseResult data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseResult data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getTimestamp(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectId().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getMeshFile().length() + 1;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getTimestamp(), cdr);
      if(data.getObjectId().length() <= 255)
      cdr.write_type_d(data.getObjectId());else
          throw new RuntimeException("object_id field exceeds the maximum length: %d > %d".formatted(data.getObjectId().length(), 255));

      if(data.getMeshFile().length() <= 255)
      cdr.write_type_d(data.getMeshFile());else
          throw new RuntimeException("mesh_file field exceeds the maximum length: %d > %d".formatted(data.getMeshFile().length(), 255));

      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
   }

   public static void read(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getTimestamp(), cdr);	
      cdr.read_type_d(data.getObjectId());	
      cdr.read_type_d(data.getMeshFile());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("timestamp", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getTimestamp());

      ser.write_type_d("object_id", data.getObjectId());
      ser.write_type_d("mesh_file", data.getMeshFile());
      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FoundationPoseResult data)
   {
      ser.read_type_a("timestamp", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getTimestamp());

      ser.read_type_d("object_id", data.getObjectId());
      ser.read_type_d("mesh_file", data.getMeshFile());
      ser.read_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

   }

   public static void staticCopy(perception_msgs.msg.dds.FoundationPoseResult src, perception_msgs.msg.dds.FoundationPoseResult dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.FoundationPoseResult createData()
   {
      return new perception_msgs.msg.dds.FoundationPoseResult();
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
   
   public void serialize(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.FoundationPoseResult src, perception_msgs.msg.dds.FoundationPoseResult dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FoundationPoseResultPubSubType newInstance()
   {
      return new FoundationPoseResultPubSubType();
   }
}
