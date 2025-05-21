package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FoundationPoseRequest" defined in "FoundationPoseRequest_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FoundationPoseRequest_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FoundationPoseRequest_.idl instead.
*
*/
public class FoundationPoseRequestPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.FoundationPoseRequest>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::FoundationPoseRequest_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c38eb7867ae9868703c4d7e7b6133a88e4b85b7d7eee5a907439a193adbaf8d4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.FoundationPoseRequest data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.FoundationPoseRequest data) throws java.io.IOException
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
      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseRequest data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseRequest data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getMeshFile().length() + 1;

      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getCdrSerializedSize(data.getColor(), current_alignment);

      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getCdrSerializedSize(data.getDepth(), current_alignment);

      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getCdrSerializedSize(data.getObjectMask(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FoundationPoseRequest data, us.ihmc.idl.CDR cdr)
   {
      if(data.getMeshFile().length() <= 255)
      cdr.write_type_d(data.getMeshFile());else
          throw new RuntimeException("mesh_file field exceeds the maximum length: %d > %d".formatted(data.getMeshFile().length(), 255));

      sensor_msgs.msg.dds.ImagePubSubType.write(data.getColor(), cdr);
      sensor_msgs.msg.dds.ImagePubSubType.write(data.getDepth(), cdr);
      sensor_msgs.msg.dds.ImagePubSubType.write(data.getObjectMask(), cdr);
   }

   public static void read(perception_msgs.msg.dds.FoundationPoseRequest data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getMeshFile());	
      sensor_msgs.msg.dds.ImagePubSubType.read(data.getColor(), cdr);	
      sensor_msgs.msg.dds.ImagePubSubType.read(data.getDepth(), cdr);	
      sensor_msgs.msg.dds.ImagePubSubType.read(data.getObjectMask(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FoundationPoseRequest data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("mesh_file", data.getMeshFile());
      ser.write_type_a("color", new sensor_msgs.msg.dds.ImagePubSubType(), data.getColor());

      ser.write_type_a("depth", new sensor_msgs.msg.dds.ImagePubSubType(), data.getDepth());

      ser.write_type_a("object_mask", new sensor_msgs.msg.dds.ImagePubSubType(), data.getObjectMask());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FoundationPoseRequest data)
   {
      ser.read_type_d("mesh_file", data.getMeshFile());
      ser.read_type_a("color", new sensor_msgs.msg.dds.ImagePubSubType(), data.getColor());

      ser.read_type_a("depth", new sensor_msgs.msg.dds.ImagePubSubType(), data.getDepth());

      ser.read_type_a("object_mask", new sensor_msgs.msg.dds.ImagePubSubType(), data.getObjectMask());

   }

   public static void staticCopy(perception_msgs.msg.dds.FoundationPoseRequest src, perception_msgs.msg.dds.FoundationPoseRequest dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.FoundationPoseRequest createData()
   {
      return new perception_msgs.msg.dds.FoundationPoseRequest();
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
   
   public void serialize(perception_msgs.msg.dds.FoundationPoseRequest data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.FoundationPoseRequest data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.FoundationPoseRequest src, perception_msgs.msg.dds.FoundationPoseRequest dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FoundationPoseRequestPubSubType newInstance()
   {
      return new FoundationPoseRequestPubSubType();
   }
}
