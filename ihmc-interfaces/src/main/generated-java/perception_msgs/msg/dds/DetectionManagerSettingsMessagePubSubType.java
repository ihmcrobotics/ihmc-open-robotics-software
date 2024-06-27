package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectionManagerSettingsMessage" defined in "DetectionManagerSettingsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectionManagerSettingsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectionManagerSettingsMessage_.idl instead.
*
*/
public class DetectionManagerSettingsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectionManagerSettingsMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectionManagerSettingsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ef4a95f16db160bad03c5270c674acdeb43fbd8cef685948b7e40762d6f80bfe";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectionManagerSettingsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectionManagerSettingsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_5(data.getMatchDistanceThreshold());

      cdr.write_type_5(data.getAcceptanceAverageConfidence());

      cdr.write_type_5(data.getStabilityAverageConfidence());

      cdr.write_type_5(data.getStabilityFrequency());

      cdr.write_type_5(data.getDetectionHistoryDuration());

   }

   public static void read(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setMatchDistanceThreshold(cdr.read_type_5());
      	
      data.setAcceptanceAverageConfidence(cdr.read_type_5());
      	
      data.setStabilityAverageConfidence(cdr.read_type_5());
      	
      data.setStabilityFrequency(cdr.read_type_5());
      	
      data.setDetectionHistoryDuration(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_5("match_distance_threshold", data.getMatchDistanceThreshold());
      ser.write_type_5("acceptance_average_confidence", data.getAcceptanceAverageConfidence());
      ser.write_type_5("stability_average_confidence", data.getStabilityAverageConfidence());
      ser.write_type_5("stability_frequency", data.getStabilityFrequency());
      ser.write_type_5("detection_history_duration", data.getDetectionHistoryDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectionManagerSettingsMessage data)
   {
      data.setMatchDistanceThreshold(ser.read_type_5("match_distance_threshold"));
      data.setAcceptanceAverageConfidence(ser.read_type_5("acceptance_average_confidence"));
      data.setStabilityAverageConfidence(ser.read_type_5("stability_average_confidence"));
      data.setStabilityFrequency(ser.read_type_5("stability_frequency"));
      data.setDetectionHistoryDuration(ser.read_type_5("detection_history_duration"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectionManagerSettingsMessage src, perception_msgs.msg.dds.DetectionManagerSettingsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectionManagerSettingsMessage createData()
   {
      return new perception_msgs.msg.dds.DetectionManagerSettingsMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectionManagerSettingsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectionManagerSettingsMessage src, perception_msgs.msg.dds.DetectionManagerSettingsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectionManagerSettingsMessagePubSubType newInstance()
   {
      return new DetectionManagerSettingsMessagePubSubType();
   }
}
