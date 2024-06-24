package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PersistentDetectionMessage" defined in "PersistentDetectionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PersistentDetectionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PersistentDetectionMessage_.idl instead.
*
*/
public class PersistentDetectionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PersistentDetectionMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PersistentDetectionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c97eeab02780af0d2ab50e7a36e594d6364a15ef9219ea6303d80ab8adc33c72";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PersistentDetectionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PersistentDetectionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 31; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.InstantDetectionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ihmc_common_msgs.msg.dds.DurationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.InstantDetectionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PersistentDetectionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PersistentDetectionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectionHistory().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.InstantDetectionMessagePubSubType.getCdrSerializedSize(data.getDetectionHistory().get(i0), current_alignment);}

      current_alignment += ihmc_common_msgs.msg.dds.DurationMessagePubSubType.getCdrSerializedSize(data.getHistoryDuration(), current_alignment);

      current_alignment += perception_msgs.msg.dds.InstantDetectionMessagePubSubType.getCdrSerializedSize(data.getFirstDetection(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PersistentDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDetectionHistory().size() <= 31)
      cdr.write_type_e(data.getDetectionHistory());else
          throw new RuntimeException("detection_history field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.DurationMessagePubSubType.write(data.getHistoryDuration(), cdr);
      perception_msgs.msg.dds.InstantDetectionMessagePubSubType.write(data.getFirstDetection(), cdr);
      cdr.write_type_6(data.getStabilityConfidenceThreshold());

      cdr.write_type_6(data.getStabilityDetectionFrequency());

      cdr.write_type_7(data.getReadyForDestruction());

   }

   public static void read(perception_msgs.msg.dds.PersistentDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getDetectionHistory());	
      ihmc_common_msgs.msg.dds.DurationMessagePubSubType.read(data.getHistoryDuration(), cdr);	
      perception_msgs.msg.dds.InstantDetectionMessagePubSubType.read(data.getFirstDetection(), cdr);	
      data.setStabilityConfidenceThreshold(cdr.read_type_6());
      	
      data.setStabilityDetectionFrequency(cdr.read_type_6());
      	
      data.setReadyForDestruction(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PersistentDetectionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("detection_history", data.getDetectionHistory());
      ser.write_type_a("history_duration", new ihmc_common_msgs.msg.dds.DurationMessagePubSubType(), data.getHistoryDuration());

      ser.write_type_a("first_detection", new perception_msgs.msg.dds.InstantDetectionMessagePubSubType(), data.getFirstDetection());

      ser.write_type_6("stability_confidence_threshold", data.getStabilityConfidenceThreshold());
      ser.write_type_6("stability_detection_frequency", data.getStabilityDetectionFrequency());
      ser.write_type_7("ready_for_destruction", data.getReadyForDestruction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PersistentDetectionMessage data)
   {
      ser.read_type_e("detection_history", data.getDetectionHistory());
      ser.read_type_a("history_duration", new ihmc_common_msgs.msg.dds.DurationMessagePubSubType(), data.getHistoryDuration());

      ser.read_type_a("first_detection", new perception_msgs.msg.dds.InstantDetectionMessagePubSubType(), data.getFirstDetection());

      data.setStabilityConfidenceThreshold(ser.read_type_6("stability_confidence_threshold"));
      data.setStabilityDetectionFrequency(ser.read_type_6("stability_detection_frequency"));
      data.setReadyForDestruction(ser.read_type_7("ready_for_destruction"));
   }

   public static void staticCopy(perception_msgs.msg.dds.PersistentDetectionMessage src, perception_msgs.msg.dds.PersistentDetectionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PersistentDetectionMessage createData()
   {
      return new perception_msgs.msg.dds.PersistentDetectionMessage();
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
   
   public void serialize(perception_msgs.msg.dds.PersistentDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PersistentDetectionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PersistentDetectionMessage src, perception_msgs.msg.dds.PersistentDetectionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PersistentDetectionMessagePubSubType newInstance()
   {
      return new PersistentDetectionMessagePubSubType();
   }
}
