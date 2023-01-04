package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedObjectMessage" defined in "DetectedObjectMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedObjectMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedObjectMessage_.idl instead.
*
*/
public class DetectedObjectMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedObjectMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedObjectMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedObjectMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedObjectMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedObjectMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedObjectMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getDetected());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToWorld(), cdr);
   }

   public static void read(perception_msgs.msg.dds.DetectedObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setDetected(cdr.read_type_7());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToWorld(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedObjectMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("detected", data.getDetected());
      ser.write_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedObjectMessage data)
   {
      data.setDetected(ser.read_type_7("detected"));
      ser.read_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedObjectMessage src, perception_msgs.msg.dds.DetectedObjectMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedObjectMessage createData()
   {
      return new perception_msgs.msg.dds.DetectedObjectMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedObjectMessage src, perception_msgs.msg.dds.DetectedObjectMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedObjectMessagePubSubType newInstance()
   {
      return new DetectedObjectMessagePubSubType();
   }
}
