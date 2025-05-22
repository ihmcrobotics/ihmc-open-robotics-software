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
   		return "a8a8140598205419fc9d2cc9c35eda27c55d51b435129465c805e4251052341e";
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectId().length() + 1;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.CDR cdr)
   {
      if(data.getObjectId().length() <= 255)
      cdr.write_type_d(data.getObjectId());else
          throw new RuntimeException("object_id field exceeds the maximum length: %d > %d".formatted(data.getObjectId().length(), 255));

      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
   }

   public static void read(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getObjectId());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FoundationPoseResult data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("object_id", data.getObjectId());
      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FoundationPoseResult data)
   {
      ser.read_type_d("object_id", data.getObjectId());
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
