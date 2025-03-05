package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedDoorListMessage" defined in "DetectedDoorListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedDoorListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedDoorListMessage_.idl instead.
*
*/
public class DetectedDoorListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedDoorListMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedDoorListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7d3fd76bdf851fa5bd90093b988884930ef98d03ba2b9a6d04653de3c02de564";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedDoorListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedDoorListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectedDoorMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedDoorListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedDoorListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectedDoors().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectedDoorMessagePubSubType.getCdrSerializedSize(data.getDetectedDoors().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedDoorListMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDetectedDoors().size() <= 16)
      cdr.write_type_e(data.getDetectedDoors());else
          throw new RuntimeException("detected_doors field exceeds the maximum length: %d > %d".formatted(data.getDetectedDoors().size(), 16));

   }

   public static void read(perception_msgs.msg.dds.DetectedDoorListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getDetectedDoors());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedDoorListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("detected_doors", data.getDetectedDoors());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedDoorListMessage data)
   {
      ser.read_type_e("detected_doors", data.getDetectedDoors());
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedDoorListMessage src, perception_msgs.msg.dds.DetectedDoorListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedDoorListMessage createData()
   {
      return new perception_msgs.msg.dds.DetectedDoorListMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedDoorListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedDoorListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedDoorListMessage src, perception_msgs.msg.dds.DetectedDoorListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedDoorListMessagePubSubType newInstance()
   {
      return new DetectedDoorListMessagePubSubType();
   }
}
