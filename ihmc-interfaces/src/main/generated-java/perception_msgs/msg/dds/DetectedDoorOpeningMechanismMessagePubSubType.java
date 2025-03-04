package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedDoorOpeningMechanismMessage" defined in "DetectedDoorOpeningMechanismMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedDoorOpeningMechanismMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedDoorOpeningMechanismMessage_.idl instead.
*
*/
public class DetectedDoorOpeningMechanismMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedDoorOpeningMechanismMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "02ac3ec78c1568b97467918714db29b451b98e92d926f9df38d74383fde687d9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data) throws java.io.IOException
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
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getName().length() <= 255)
      cdr.write_type_d(data.getName());else
          throw new RuntimeException("name field exceeds the maximum length");

      cdr.write_type_9(data.getSide());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToWorld(), cdr);
   }

   public static void read(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getName());	
      data.setSide(cdr.read_type_9());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToWorld(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("name", data.getName());
      ser.write_type_9("side", data.getSide());
      ser.write_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data)
   {
      ser.read_type_d("name", data.getName());
      data.setSide(ser.read_type_9("side"));
      ser.read_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage src, perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage createData()
   {
      return new perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage src, perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedDoorOpeningMechanismMessagePubSubType newInstance()
   {
      return new DetectedDoorOpeningMechanismMessagePubSubType();
   }
}
